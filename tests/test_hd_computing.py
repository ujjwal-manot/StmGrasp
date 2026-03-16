"""
Tests for Hyperdimensional Computing Slip Detection
Ported from esp32_brain/hd_slip.cpp and hd_slip.h

Binary hypervectors (D=2048 bits = 256 bytes).
7 features: 3x FSR AC-RMS, 3x FSR DC, dF/dt.
Level encoding -> XOR binding -> Hamming distance classification.
Two classes: STABLE vs SLIP.
"""

import math
import pytest
import numpy as np

# ---------------------------------------------------------------------------
# Constants ported from hd_slip.h
# ---------------------------------------------------------------------------

HD_DIM = 2048
HD_BYTES = HD_DIM // 8  # 256

HD_NUM_FEATURES = 7
HD_FEAT_FSR1_AC = 0
HD_FEAT_FSR2_AC = 1
HD_FEAT_FSR3_AC = 2
HD_FEAT_FSR1_DC = 3
HD_FEAT_FSR2_DC = 4
HD_FEAT_FSR3_DC = 5
HD_FEAT_DFDT = 6

HD_NUM_LEVELS = 16
HD_NUM_CLASSES = 2
HD_CLASS_STABLE = 0
HD_CLASS_SLIP = 1
HD_MIN_TRAIN = 5
HD_SEED = 0xDEADBEEF


# ---------------------------------------------------------------------------
# Python port of the HD computing pipeline
# ---------------------------------------------------------------------------

class XorShift32:
    """Port of the xorshift32 PRNG from hd_slip.cpp."""

    def __init__(self, seed):
        self.state = seed & 0xFFFFFFFF
        if self.state == 0:
            self.state = 1

    def next(self):
        x = self.state
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= (x >> 17)
        x ^= (x << 5) & 0xFFFFFFFF
        self.state = x & 0xFFFFFFFF
        return self.state


def random_vector(prng):
    """Generate a random binary hypervector using the PRNG."""
    vec = bytearray(HD_BYTES)
    for i in range(0, HD_BYTES, 4):
        r = prng.next()
        vec[i] = r & 0xFF
        vec[i + 1] = (r >> 8) & 0xFF
        vec[i + 2] = (r >> 16) & 0xFF
        if i + 3 < HD_BYTES:
            vec[i + 3] = (r >> 24) & 0xFF
    return bytes(vec)


def circular_shift(src, shift):
    """Circular bit-shift a binary vector by 'shift' positions."""
    shift = shift % HD_DIM
    if shift < 0:
        shift += HD_DIM
    if shift == 0:
        return bytes(src)

    dst = bytearray(HD_BYTES)
    byte_shift = shift // 8
    bit_shift = shift % 8

    for i in range(HD_BYTES):
        src_byte1 = (i + byte_shift) % HD_BYTES
        src_byte2 = (i + byte_shift + 1) % HD_BYTES
        if bit_shift == 0:
            dst[i] = src[src_byte1]
        else:
            dst[i] = ((src[src_byte1] << bit_shift) | (src[src_byte2] >> (8 - bit_shift))) & 0xFF

    return bytes(dst)


def xor_bind(a, b):
    """XOR two binary vectors."""
    return bytes(x ^ y for x, y in zip(a, b))


def hamming_distance(a, b):
    """Hamming distance between two binary vectors."""
    dist = 0
    for x, y in zip(a, b):
        diff = x ^ y
        while diff:
            diff &= diff - 1
            dist += 1
    return dist


def popcount_vector(vec):
    """Count total number of 1-bits in a vector."""
    count = 0
    for byte in vec:
        while byte:
            byte &= byte - 1
            count += 1
    return count


class HDSlipModel:
    """
    Python port of the HD computing slip detection model from hd_slip.cpp.
    """

    def __init__(self):
        # Initialize PRNG and generate seed vectors
        prng = XorShift32(HD_SEED)
        self.seed_vectors = [random_vector(prng) for _ in range(HD_NUM_FEATURES)]

        # Class accumulators (int16 per bit position per class)
        self.class_accum = [[0] * HD_DIM for _ in range(HD_NUM_CLASSES)]

        # Binary class prototype vectors
        self.class_vectors = [bytes(HD_BYTES) for _ in range(HD_NUM_CLASSES)]

        # Training counts
        self.train_count = [0] * HD_NUM_CLASSES

        # Feature normalization ranges (initialized to defaults)
        self.feat_min = [
            0.0, 0.0, 0.0,       # FSR AC RMS: 0 to 2.0 N
            0.0, 0.0, 0.0,       # FSR DC: 0 to 20 N
            -10.0,                # dF/dt: -10 to 10 N/s
        ]
        self.feat_max = [
            2.0, 2.0, 2.0,
            20.0, 20.0, 20.0,
            10.0,
        ]
        self.norm_initialized = True

    def _quantize(self, value, feat_idx):
        """Quantize a feature value to a level index [0, HD_NUM_LEVELS-1]."""
        if not self.norm_initialized:
            return HD_NUM_LEVELS // 2

        feat_range = self.feat_max[feat_idx] - self.feat_min[feat_idx]
        if feat_range < 1e-6:
            return HD_NUM_LEVELS // 2

        normalized = (value - self.feat_min[feat_idx]) / feat_range
        normalized = min(max(normalized, 0.0), 1.0)
        level = int(normalized * (HD_NUM_LEVELS - 1) + 0.5)
        return level

    def _encode_features(self, features):
        """Encode a feature vector into a query hypervector."""
        # Start with first feature's level vector
        level = self._quantize(features[0], 0)
        shift = level * (HD_DIM // HD_NUM_LEVELS)
        query = circular_shift(self.seed_vectors[0], shift)

        # XOR-bind with each subsequent feature's level vector
        for f in range(1, HD_NUM_FEATURES):
            level = self._quantize(features[f], f)
            shift = level * (HD_DIM // HD_NUM_LEVELS)
            temp = circular_shift(self.seed_vectors[f], shift)
            query = xor_bind(query, temp)

        return query

    def _threshold_class(self, class_idx):
        """Majority-vote threshold accumulators to produce binary class vector."""
        count = self.train_count[class_idx]
        if count == 0:
            self.class_vectors[class_idx] = bytes(HD_BYTES)
            return

        threshold = count // 2  # integer division matches C int16_t cast
        vec = bytearray(HD_BYTES)

        for i in range(HD_DIM):
            byte_idx = i // 8
            bit_idx = 7 - (i % 8)
            if self.class_accum[class_idx][i] > threshold:
                vec[byte_idx] |= (1 << bit_idx)

        self.class_vectors[class_idx] = bytes(vec)

    def train(self, features, label):
        """Add a training example."""
        if label >= HD_NUM_CLASSES:
            return

        # Update normalization
        for i in range(HD_NUM_FEATURES):
            if features[i] < self.feat_min[i]:
                self.feat_min[i] = features[i]
            if features[i] > self.feat_max[i]:
                self.feat_max[i] = features[i]

        # Encode
        encoded = self._encode_features(features)

        # Accumulate
        for i in range(HD_DIM):
            byte_idx = i // 8
            bit_idx = 7 - (i % 8)
            if encoded[byte_idx] & (1 << bit_idx):
                self.class_accum[label][i] += 1

        self.train_count[label] += 1
        self._threshold_class(label)

    def is_ready(self):
        """Check if model has enough training data."""
        return (self.train_count[HD_CLASS_STABLE] >= HD_MIN_TRAIN and
                self.train_count[HD_CLASS_SLIP] >= HD_MIN_TRAIN)

    def classify(self, features):
        """Classify a feature vector."""
        result = {
            "predicted_class": HD_CLASS_STABLE,
            "confidence": 0.0,
            "hamming_stable": HD_DIM // 2,
            "hamming_slip": HD_DIM // 2,
            "valid": False,
        }

        if not self.is_ready():
            return result

        query = self._encode_features(features)

        result["hamming_stable"] = hamming_distance(query, self.class_vectors[HD_CLASS_STABLE])
        result["hamming_slip"] = hamming_distance(query, self.class_vectors[HD_CLASS_SLIP])

        if result["hamming_slip"] < result["hamming_stable"]:
            result["predicted_class"] = HD_CLASS_SLIP
        else:
            result["predicted_class"] = HD_CLASS_STABLE

        margin = result["hamming_stable"] - result["hamming_slip"]
        abs_margin = abs(margin)
        result["confidence"] = min(abs_margin / (HD_DIM / 4), 1.0)
        result["valid"] = True

        return result

    def get_train_count(self, label):
        if label >= HD_NUM_CLASSES:
            return 0
        return self.train_count[label]

    def reset_training(self):
        self.class_accum = [[0] * HD_DIM for _ in range(HD_NUM_CLASSES)]
        self.class_vectors = [bytes(HD_BYTES) for _ in range(HD_NUM_CLASSES)]
        self.train_count = [0] * HD_NUM_CLASSES


# ---------------------------------------------------------------------------
# Training data generators
# ---------------------------------------------------------------------------

def make_stable_features(rng=None):
    """Generate a feature vector typical of stable grip (low AC, steady DC, low dF/dt)."""
    if rng is None:
        rng = np.random.default_rng(42)
    return [
        rng.uniform(0.0, 0.1),   # FSR1 AC RMS (low vibration)
        rng.uniform(0.0, 0.1),   # FSR2 AC RMS
        rng.uniform(0.0, 0.1),   # FSR3 AC RMS
        rng.uniform(3.0, 8.0),   # FSR1 DC (good contact force)
        rng.uniform(3.0, 8.0),   # FSR2 DC
        rng.uniform(3.0, 8.0),   # FSR3 DC
        rng.uniform(-0.5, 0.5),  # dF/dt (stable)
    ]


def make_slip_features(rng=None):
    """Generate a feature vector typical of slip (high AC, varying DC, large dF/dt)."""
    if rng is None:
        rng = np.random.default_rng(42)
    return [
        rng.uniform(0.5, 2.0),    # FSR1 AC RMS (high vibration)
        rng.uniform(0.5, 2.0),    # FSR2 AC RMS
        rng.uniform(0.5, 2.0),    # FSR3 AC RMS
        rng.uniform(0.5, 3.0),    # FSR1 DC (losing contact)
        rng.uniform(0.5, 3.0),    # FSR2 DC
        rng.uniform(0.5, 3.0),    # FSR3 DC
        rng.uniform(-8.0, -3.0),  # dF/dt (force dropping fast)
    ]


# ===========================================================================
# TESTS
# ===========================================================================


class TestHypervectorBasics:
    """Test fundamental hypervector operations."""

    def test_seed_vector_dimensions(self):
        """Seed vectors should be HD_BYTES (256) bytes long."""
        model = HDSlipModel()
        for sv in model.seed_vectors:
            assert len(sv) == HD_BYTES

    def test_seed_vectors_approximately_half_ones(self):
        """Random seed vectors should have approximately 50% 1-bits."""
        model = HDSlipModel()
        for sv in model.seed_vectors:
            ones = popcount_vector(sv)
            # With D=2048, we expect ~1024 ones; allow generous tolerance
            assert 800 < ones < 1250, f"Expected ~1024 ones, got {ones}"

    def test_seed_vectors_are_quasi_orthogonal(self):
        """Different seed vectors should be approximately orthogonal (Hamming ~D/2)."""
        model = HDSlipModel()
        for i in range(HD_NUM_FEATURES):
            for j in range(i + 1, HD_NUM_FEATURES):
                dist = hamming_distance(model.seed_vectors[i], model.seed_vectors[j])
                # Expect ~D/2 = 1024; allow generous tolerance
                assert 700 < dist < 1350, \
                    f"Seed vectors {i} and {j} have Hamming distance {dist}, expected ~{HD_DIM // 2}"

    def test_xor_self_is_zero(self):
        """XOR of a vector with itself should produce the zero vector."""
        model = HDSlipModel()
        zero = xor_bind(model.seed_vectors[0], model.seed_vectors[0])
        assert all(b == 0 for b in zero)

    def test_xor_is_commutative(self):
        """XOR binding should be commutative."""
        model = HDSlipModel()
        a, b = model.seed_vectors[0], model.seed_vectors[1]
        assert xor_bind(a, b) == xor_bind(b, a)

    def test_circular_shift_identity(self):
        """Shift by 0 should return the same vector."""
        model = HDSlipModel()
        sv = model.seed_vectors[0]
        assert circular_shift(sv, 0) == sv

    def test_circular_shift_full_rotation(self):
        """Shift by HD_DIM should return the same vector."""
        model = HDSlipModel()
        sv = model.seed_vectors[0]
        assert circular_shift(sv, HD_DIM) == sv

    def test_circular_shift_preserves_popcount(self):
        """Circular shift should not change the number of 1-bits."""
        model = HDSlipModel()
        sv = model.seed_vectors[0]
        original_ones = popcount_vector(sv)
        for shift in [1, 7, 8, 128, 1000, HD_DIM - 1]:
            shifted = circular_shift(sv, shift)
            shifted_ones = popcount_vector(shifted)
            assert shifted_ones == original_ones, \
                f"Shift by {shift}: {shifted_ones} ones vs original {original_ones}"

    def test_hamming_distance_self_is_zero(self):
        """Hamming distance of a vector with itself should be 0."""
        model = HDSlipModel()
        assert hamming_distance(model.seed_vectors[0], model.seed_vectors[0]) == 0

    def test_hamming_distance_symmetric(self):
        """Hamming distance should be symmetric."""
        model = HDSlipModel()
        a, b = model.seed_vectors[0], model.seed_vectors[1]
        assert hamming_distance(a, b) == hamming_distance(b, a)


class TestEncoding:
    """Test the feature encoding pipeline."""

    def test_encode_produces_correct_dimensions(self):
        """Encoded vector should have HD_BYTES length."""
        model = HDSlipModel()
        features = [0.1, 0.1, 0.1, 5.0, 5.0, 5.0, 0.0]
        query = model._encode_features(features)
        assert len(query) == HD_BYTES

    def test_same_features_same_encoding(self):
        """Identical features should produce identical encodings."""
        model = HDSlipModel()
        features = [0.1, 0.1, 0.1, 5.0, 5.0, 5.0, 0.0]
        q1 = model._encode_features(features)
        q2 = model._encode_features(features)
        assert q1 == q2

    def test_different_features_different_encoding(self):
        """Significantly different features should produce different encodings."""
        model = HDSlipModel()
        stable = [0.05, 0.05, 0.05, 6.0, 6.0, 6.0, 0.0]
        slip = [1.5, 1.5, 1.5, 1.0, 1.0, 1.0, -7.0]
        q_stable = model._encode_features(stable)
        q_slip = model._encode_features(slip)
        assert q_stable != q_slip
        # They should be somewhat different
        dist = hamming_distance(q_stable, q_slip)
        assert dist > 100, f"Expected significant difference, got Hamming distance {dist}"

    def test_similar_features_closer_encoding(self):
        """Similar features should produce encodings with smaller Hamming distance."""
        model = HDSlipModel()
        base = [0.1, 0.1, 0.1, 5.0, 5.0, 5.0, 0.0]
        similar = [0.12, 0.11, 0.09, 5.1, 4.9, 5.0, 0.1]
        very_different = [1.8, 1.9, 1.7, 1.0, 0.5, 1.5, -9.0]

        q_base = model._encode_features(base)
        q_similar = model._encode_features(similar)
        q_diff = model._encode_features(very_different)

        dist_similar = hamming_distance(q_base, q_similar)
        dist_diff = hamming_distance(q_base, q_diff)

        assert dist_similar < dist_diff, \
            f"Similar distance ({dist_similar}) should be less than different distance ({dist_diff})"


class TestTraining:
    """Test the training pipeline."""

    def test_training_increments_count(self):
        """Training should increment the class count."""
        model = HDSlipModel()
        assert model.get_train_count(HD_CLASS_STABLE) == 0
        assert model.get_train_count(HD_CLASS_SLIP) == 0

        rng = np.random.default_rng(42)
        model.train(make_stable_features(rng), HD_CLASS_STABLE)
        assert model.get_train_count(HD_CLASS_STABLE) == 1
        assert model.get_train_count(HD_CLASS_SLIP) == 0

    def test_not_ready_before_min_train(self):
        """Model should not be ready until HD_MIN_TRAIN examples per class."""
        model = HDSlipModel()
        rng = np.random.default_rng(42)

        for i in range(HD_MIN_TRAIN - 1):
            model.train(make_stable_features(rng), HD_CLASS_STABLE)
            model.train(make_slip_features(rng), HD_CLASS_SLIP)

        assert not model.is_ready()

    def test_ready_after_min_train(self):
        """Model should be ready after HD_MIN_TRAIN examples per class."""
        model = HDSlipModel()
        rng = np.random.default_rng(42)

        for i in range(HD_MIN_TRAIN):
            model.train(make_stable_features(rng), HD_CLASS_STABLE)
            model.train(make_slip_features(rng), HD_CLASS_SLIP)

        assert model.is_ready()

    def test_reset_clears_training(self):
        """Reset should clear all training data."""
        model = HDSlipModel()
        rng = np.random.default_rng(42)

        for i in range(10):
            model.train(make_stable_features(rng), HD_CLASS_STABLE)
            model.train(make_slip_features(rng), HD_CLASS_SLIP)

        assert model.is_ready()
        model.reset_training()
        assert not model.is_ready()
        assert model.get_train_count(HD_CLASS_STABLE) == 0
        assert model.get_train_count(HD_CLASS_SLIP) == 0

    def test_invalid_label_ignored(self):
        """Training with invalid label (>= HD_NUM_CLASSES) should be ignored."""
        model = HDSlipModel()
        rng = np.random.default_rng(42)
        model.train(make_stable_features(rng), 99)  # invalid
        assert model.get_train_count(0) == 0
        assert model.get_train_count(1) == 0


class TestClassification:
    """Test classification pipeline correctness."""

    @pytest.fixture
    def trained_model(self):
        """Create a model trained with 100 examples per class."""
        model = HDSlipModel()
        rng_stable = np.random.default_rng(100)
        rng_slip = np.random.default_rng(200)

        for _ in range(100):
            model.train(make_stable_features(rng_stable), HD_CLASS_STABLE)
            model.train(make_slip_features(rng_slip), HD_CLASS_SLIP)

        assert model.is_ready()
        return model

    def test_classify_returns_invalid_when_not_ready(self):
        """Classification should return invalid if model is not trained."""
        model = HDSlipModel()
        features = [0.1, 0.1, 0.1, 5.0, 5.0, 5.0, 0.0]
        result = model.classify(features)
        assert not result["valid"]

    def test_classify_returns_valid_result(self, trained_model):
        """Classification of any valid features should return a valid result."""
        features = [0.1, 0.1, 0.1, 5.0, 5.0, 5.0, 0.0]
        result = trained_model.classify(features)
        assert result["valid"]
        assert result["predicted_class"] in (HD_CLASS_STABLE, HD_CLASS_SLIP)
        assert 0 <= result["confidence"] <= 1.0
        assert 0 <= result["hamming_stable"] <= HD_DIM
        assert 0 <= result["hamming_slip"] <= HD_DIM

    def test_confidence_is_bounded(self, trained_model):
        """Confidence should always be between 0 and 1."""
        rng = np.random.default_rng(999)
        for _ in range(50):
            features = make_stable_features(rng)
            result = trained_model.classify(features)
            assert 0.0 <= result["confidence"] <= 1.0

    def test_identical_training_produces_correct_classification(self):
        """When trained with fixed (non-noisy) features, model should classify correctly."""
        model = HDSlipModel()

        # Use perfectly fixed features for each class (no noise)
        stable_features = [0.0, 0.0, 0.0, 10.0, 10.0, 10.0, 0.0]
        slip_features = [2.0, 2.0, 2.0, 0.0, 0.0, 0.0, -10.0]

        for _ in range(HD_MIN_TRAIN + 5):
            model.train(stable_features, HD_CLASS_STABLE)
            model.train(slip_features, HD_CLASS_SLIP)

        assert model.is_ready()

        # Classify the exact training patterns
        result_stable = model.classify(stable_features)
        result_slip = model.classify(slip_features)

        assert result_stable["valid"]
        assert result_slip["valid"]
        assert result_stable["predicted_class"] == HD_CLASS_STABLE, \
            f"Expected STABLE, got {result_stable['predicted_class']}"
        assert result_slip["predicted_class"] == HD_CLASS_SLIP, \
            f"Expected SLIP, got {result_slip['predicted_class']}"

    def test_hamming_distance_consistent_with_prediction(self, trained_model):
        """The predicted class should match the closer prototype by Hamming distance."""
        rng = np.random.default_rng(42)
        for _ in range(30):
            features = make_stable_features(rng)
            result = trained_model.classify(features)
            if result["hamming_slip"] < result["hamming_stable"]:
                assert result["predicted_class"] == HD_CLASS_SLIP
            elif result["hamming_stable"] < result["hamming_slip"]:
                assert result["predicted_class"] == HD_CLASS_STABLE
            # Equal distances -> defaults to STABLE (matches C code)


class TestHammingDistanceProperties:
    """Test that Hamming distances follow expected patterns after training."""

    @pytest.fixture
    def trained_model(self):
        model = HDSlipModel()
        rng_stable = np.random.default_rng(100)
        rng_slip = np.random.default_rng(200)

        for _ in range(100):
            model.train(make_stable_features(rng_stable), HD_CLASS_STABLE)
            model.train(make_slip_features(rng_slip), HD_CLASS_SLIP)

        return model

    def test_class_prototypes_are_different(self, trained_model):
        """The two class prototypes should be significantly different."""
        dist = hamming_distance(
            trained_model.class_vectors[HD_CLASS_STABLE],
            trained_model.class_vectors[HD_CLASS_SLIP],
        )
        # They should not be identical or near-identical
        assert dist > 100, \
            f"Class prototypes too similar: Hamming distance = {dist}"

    def test_hamming_distances_are_reasonable(self, trained_model):
        """Hamming distances to prototypes should be roughly in [0, D] range."""
        rng = np.random.default_rng(500)
        for _ in range(20):
            features = make_stable_features(rng)
            result = trained_model.classify(features)
            # Both distances should be > 0 and < HD_DIM
            assert 0 < result["hamming_stable"] < HD_DIM
            assert 0 < result["hamming_slip"] < HD_DIM

    def test_fixed_features_correct_distance_ordering(self):
        """
        With deterministic training on fixed features, the training examples
        themselves should be closer to their own class prototype.
        """
        model = HDSlipModel()
        stable = [0.0, 0.0, 0.0, 10.0, 10.0, 10.0, 0.0]
        slip = [2.0, 2.0, 2.0, 0.0, 0.0, 0.0, -10.0]

        for _ in range(20):
            model.train(stable, HD_CLASS_STABLE)
            model.train(slip, HD_CLASS_SLIP)

        res_stable = model.classify(stable)
        res_slip = model.classify(slip)

        assert res_stable["hamming_stable"] < res_stable["hamming_slip"], \
            "Stable training data should be closer to STABLE prototype"
        assert res_slip["hamming_slip"] < res_slip["hamming_stable"], \
            "Slip training data should be closer to SLIP prototype"


class TestMajorityVoteBundling:
    """Test that majority-vote bundling produces correct class prototypes."""

    def test_majority_vote_with_identical_vectors(self):
        """If all training examples encode identically, prototype should match."""
        model = HDSlipModel()
        # Use identical features every time
        features = [0.1, 0.1, 0.1, 5.0, 5.0, 5.0, 0.0]

        for _ in range(10):
            model.train(features, HD_CLASS_STABLE)

        # The class prototype should be very close to the encoded features
        query = model._encode_features(features)
        dist = hamming_distance(query, model.class_vectors[HD_CLASS_STABLE])
        # With identical training, should be exact (or very close due to integer threshold)
        assert dist <= 10, \
            f"Identical training should produce near-identical prototype, got distance {dist}"

    def test_class_vector_popcount_reasonable(self):
        """Class prototype vectors should have a reasonable number of 1-bits."""
        model = HDSlipModel()
        rng = np.random.default_rng(42)

        for _ in range(20):
            model.train(make_stable_features(rng), HD_CLASS_STABLE)
            model.train(make_slip_features(rng), HD_CLASS_SLIP)

        for cls in range(HD_NUM_CLASSES):
            ones = popcount_vector(model.class_vectors[cls])
            # Should be roughly D/2 = 1024, but there's variance
            assert 500 < ones < 1550, \
                f"Class {cls} prototype has {ones} 1-bits, expected roughly ~{HD_DIM // 2}"


class TestTrainingDataClassification:
    """Test that the model can correctly classify its own training data."""

    def test_classify_training_data(self):
        """Classification of training data should be mostly correct."""
        model = HDSlipModel()
        rng = np.random.default_rng(42)

        training_stable = []
        training_slip = []

        for _ in range(30):
            f_stable = make_stable_features(rng)
            f_slip = make_slip_features(rng)
            training_stable.append(f_stable)
            training_slip.append(f_slip)
            model.train(f_stable, HD_CLASS_STABLE)
            model.train(f_slip, HD_CLASS_SLIP)

        assert model.is_ready()

        # Classify the training data itself
        correct_stable = sum(
            1 for f in training_stable
            if model.classify(f)["predicted_class"] == HD_CLASS_STABLE
        )
        correct_slip = sum(
            1 for f in training_slip
            if model.classify(f)["predicted_class"] == HD_CLASS_SLIP
        )

        total_correct = correct_stable + correct_slip
        total = len(training_stable) + len(training_slip)
        accuracy = total_correct / total

        assert accuracy >= 0.7, \
            f"Training data accuracy {accuracy:.1%} is too low (expected >= 70%)"
