"""
Tests for Dempster-Shafer Evidence Theory Fusion
Ported from esp32_brain/ds_fusion.cpp and ds_fusion.h

Frame of discernment Theta = {Metal, Skin, Plastic, Wood, Glass, Cardboard}
Represented as bitmasks: bit 0 = Metal, bit 1 = Skin, ..., bit 5 = Cardboard
"""

import math
import pytest

# ---------------------------------------------------------------------------
# Constants ported from config.h and ds_fusion.h
# ---------------------------------------------------------------------------

MATERIAL_COUNT = 6
MAT_METAL = 0
MAT_SKIN = 1
MAT_PLASTIC = 2
MAT_WOOD = 3
MAT_GLASS = 4
MAT_CARDBOARD = 5
MAT_UNKNOWN = 255

DS_THETA = (1 << MATERIAL_COUNT) - 1  # 0b00111111 = 63
DS_EMPTY = 0
DS_MAX_FOCAL = 10
DS_MASS_EPSILON = 0.001
DS_MAX_IGNORANCE = 0.95

# Material database from config.h
MATERIAL_DB = [
    {"name": "Metal",     "impedance_magnitude": 1.0,      "impedance_phase_deg": 0.0,   "max_grip_force_N": 8.0, "slip_risk": 0.2},
    {"name": "Skin",      "impedance_magnitude": 50000.0,  "impedance_phase_deg": -35.0, "max_grip_force_N": 1.5, "slip_risk": 0.3},
    {"name": "Plastic",   "impedance_magnitude": 1000000.0,"impedance_phase_deg": -85.0, "max_grip_force_N": 5.0, "slip_risk": 0.6},
    {"name": "Wood",      "impedance_magnitude": 500000.0, "impedance_phase_deg": -20.0, "max_grip_force_N": 6.0, "slip_risk": 0.3},
    {"name": "Glass",     "impedance_magnitude": 10000000.0,"impedance_phase_deg": -88.0,"max_grip_force_N": 4.0, "slip_risk": 0.8},
    {"name": "Cardboard", "impedance_magnitude": 200000.0, "impedance_phase_deg": -25.0, "max_grip_force_N": 2.0, "slip_risk": 0.4},
]

# Gaussian material models from ds_fusion.cpp
MATERIAL_MODELS = [
    # Metal
    {"imp_log_mag_mean": 0.0,  "imp_log_mag_std": 0.6,
     "imp_phase_mean": 0.0,    "imp_phase_std": 10.0,
     "aco_freq_mean": 1800.0,  "aco_freq_std": 600.0,
     "aco_decay_mean": 0.35,   "aco_decay_std": 0.12,
     "compatible_geometries": 0b11111},
    # Skin
    {"imp_log_mag_mean": 4.7,  "imp_log_mag_std": 0.5,
     "imp_phase_mean": -35.0,  "imp_phase_std": 12.0,
     "aco_freq_mean": 100.0,   "aco_freq_std": 80.0,
     "aco_decay_mean": 0.01,   "aco_decay_std": 0.02,
     "compatible_geometries": 0b10011},
    # Plastic
    {"imp_log_mag_mean": 6.0,  "imp_log_mag_std": 0.5,
     "imp_phase_mean": -85.0,  "imp_phase_std": 8.0,
     "aco_freq_mean": 900.0,   "aco_freq_std": 400.0,
     "aco_decay_mean": 0.12,   "aco_decay_std": 0.08,
     "compatible_geometries": 0b11111},
    # Wood
    {"imp_log_mag_mean": 5.7,  "imp_log_mag_std": 0.5,
     "imp_phase_mean": -20.0,  "imp_phase_std": 12.0,
     "aco_freq_mean": 350.0,   "aco_freq_std": 200.0,
     "aco_decay_mean": 0.06,   "aco_decay_std": 0.05,
     "compatible_geometries": 0b11001},
    # Glass
    {"imp_log_mag_mean": 7.0,  "imp_log_mag_std": 0.4,
     "imp_phase_mean": -88.0,  "imp_phase_std": 5.0,
     "aco_freq_mean": 2800.0,  "aco_freq_std": 800.0,
     "aco_decay_mean": 0.30,   "aco_decay_std": 0.10,
     "compatible_geometries": 0b10011},
    # Cardboard
    {"imp_log_mag_mean": 5.3,  "imp_log_mag_std": 0.5,
     "imp_phase_mean": -25.0,  "imp_phase_std": 12.0,
     "aco_freq_mean": 200.0,   "aco_freq_std": 120.0,
     "aco_decay_mean": 0.03,   "aco_decay_std": 0.03,
     "compatible_geometries": 0b01001},
]


# ---------------------------------------------------------------------------
# Python port of the DS fusion logic
# ---------------------------------------------------------------------------

def popcount(x):
    """Count set bits in an integer."""
    count = 0
    while x:
        count += x & 1
        x >>= 1
    return count


def gaussian_log_likelihood(x, mean, std):
    """Gaussian log-likelihood (unnormalized), matching C code."""
    if std < 0.001:
        std = 0.001
    d = (x - mean) / std
    return -0.5 * d * d


def likelihoods_to_mass(likelihoods, confidence):
    """
    Convert log-likelihoods to a mass function (BPA).
    Returns list of (subset, mass) tuples and the total count.
    """
    elements = []

    # Find max log-likelihood for numerical stability
    max_ll = max(likelihoods)

    # Convert to normalized probabilities
    probs = [math.exp(ll - max_ll) for ll in likelihoods]
    s = sum(probs)
    if s < 1e-12:
        s = 1e-12
    probs = [p / s for p in probs]

    # Allocate mass
    sensor_mass = min(max(confidence, 0.0), 1.0 - DS_MASS_EPSILON)
    ignorance = 1.0 - sensor_mass

    # Assign mass to each singleton
    for i in range(MATERIAL_COUNT):
        m = probs[i] * sensor_mass
        if m > DS_MASS_EPSILON:
            elements.append({"subset": 1 << i, "mass": m})

    # Assign remaining mass to Theta (ignorance)
    if ignorance > DS_MASS_EPSILON:
        elements.append({"subset": DS_THETA, "mass": ignorance})

    return elements


def ds_from_impedance(magnitude, phase_deg, confidence):
    """Create mass function from impedance data."""
    mf = {"elements": [], "conflict": 0.0}

    if confidence < 0.05:
        mf["elements"] = [{"subset": DS_THETA, "mass": 1.0}]
        return mf

    log_mag = math.log10(max(magnitude, 0.01))

    ll = []
    for i in range(MATERIAL_COUNT):
        m = MATERIAL_MODELS[i]
        ll_mag = gaussian_log_likelihood(log_mag, m["imp_log_mag_mean"], m["imp_log_mag_std"])
        ll_phase = gaussian_log_likelihood(phase_deg, m["imp_phase_mean"], m["imp_phase_std"])
        ll.append(ll_mag + ll_phase)

    mf["elements"] = likelihoods_to_mass(ll, confidence * 0.85)
    return mf


def ds_from_acoustic(dominant_freq, decay_ratio, confidence):
    """Create mass function from acoustic tap data."""
    mf = {"elements": [], "conflict": 0.0}

    if confidence < 0.05:
        mf["elements"] = [{"subset": DS_THETA, "mass": 1.0}]
        return mf

    ll = []
    for i in range(MATERIAL_COUNT):
        m = MATERIAL_MODELS[i]
        ll_freq = gaussian_log_likelihood(dominant_freq, m["aco_freq_mean"], m["aco_freq_std"])
        ll_decay = gaussian_log_likelihood(decay_ratio, m["aco_decay_mean"], m["aco_decay_std"])
        ll.append(ll_freq + ll_decay)

    mf["elements"] = likelihoods_to_mass(ll, confidence * 0.70)
    return mf


def ds_from_curvature(geometry, flatness, sharp_edge):
    """Create mass function from curvature data."""
    mf = {"elements": [], "conflict": 0.0}

    # Build compatible materials for this geometry
    compatible = DS_EMPTY
    for i in range(MATERIAL_COUNT):
        if MATERIAL_MODELS[i]["compatible_geometries"] & (1 << geometry):
            compatible |= (1 << i)

    if compatible == DS_EMPTY:
        compatible = DS_THETA

    num_compatible = popcount(compatible)
    discrimination = 1.0 - (num_compatible / MATERIAL_COUNT)
    curv_confidence = discrimination * 0.5 + flatness * 0.15
    if sharp_edge:
        curv_confidence += 0.1
    curv_confidence = min(curv_confidence, 0.6)

    if num_compatible < MATERIAL_COUNT:
        mf["elements"] = [
            {"subset": compatible, "mass": curv_confidence},
            {"subset": DS_THETA, "mass": 1.0 - curv_confidence},
        ]
    else:
        mf["elements"] = [{"subset": DS_THETA, "mass": 1.0}]

    return mf


def ds_combine(m1, m2):
    """Combine two mass functions using Dempster's rule."""
    combined_mass = [0.0] * 64  # 2^6 subsets
    conflict = 0.0

    for e1 in m1["elements"]:
        for e2 in m2["elements"]:
            intersection = e1["subset"] & e2["subset"]
            product = e1["mass"] * e2["mass"]

            if intersection == DS_EMPTY:
                conflict += product
            else:
                combined_mass[intersection] += product

    result = {"elements": [], "conflict": conflict}

    norm = 1.0 - conflict
    if norm < 0.001:
        # Total conflict - return vacuous mass
        result["elements"] = [{"subset": DS_THETA, "mass": 1.0}]
        return result

    inv_norm = 1.0 / norm

    for s in range(1, DS_THETA + 1):
        m = combined_mass[s] * inv_norm
        if m > DS_MASS_EPSILON and len(result["elements"]) < DS_MAX_FOCAL:
            result["elements"].append({"subset": s, "mass": m})

    return result


def ds_combine_all(mass_functions):
    """Combine a list of mass functions sequentially."""
    if not mass_functions:
        return {"elements": [{"subset": DS_THETA, "mass": 1.0}], "conflict": 0.0}

    accumulated = mass_functions[0]
    for i in range(1, len(mass_functions)):
        accumulated = ds_combine(accumulated, mass_functions[i])
    return accumulated


def ds_get_result(combined):
    """Extract belief, plausibility, best material from combined mass function."""
    belief = [0.0] * MATERIAL_COUNT
    plausibility = [0.0] * MATERIAL_COUNT

    for mat in range(MATERIAL_COUNT):
        singleton = 1 << mat
        bel = 0.0
        pl = 0.0

        for e in combined["elements"]:
            a = e["subset"]
            m = e["mass"]

            # Belief: only the singleton itself qualifies for a singleton query
            if a == singleton:
                bel += m

            # Plausibility: any subset that intersects the singleton
            if a & singleton:
                pl += m

        belief[mat] = bel
        plausibility[mat] = pl

    # Residual ignorance (mass on multi-element sets)
    mass_theta = 0.0
    for e in combined["elements"]:
        if popcount(e["subset"]) > 1:
            mass_theta += e["mass"]

    conflict = combined["conflict"]

    # Find best material by belief
    best_bel = 0.0
    best_idx = MAT_UNKNOWN
    for i in range(MATERIAL_COUNT):
        if belief[i] > best_bel:
            best_bel = belief[i]
            best_idx = i

    best_plausibility = plausibility[best_idx] if best_idx < MATERIAL_COUNT else 0.0
    valid = best_idx < MATERIAL_COUNT and best_bel > 0.1

    return {
        "belief": belief,
        "plausibility": plausibility,
        "mass_singleton": belief[:],  # for singletons, Bel = direct mass
        "mass_theta": mass_theta,
        "conflict": conflict,
        "best_material": best_idx,
        "best_belief": best_bel,
        "best_plausibility": best_plausibility,
        "valid": valid,
    }


# ---------------------------------------------------------------------------
# Helper to sum all masses in a mass function (should always be ~1.0)
# ---------------------------------------------------------------------------

def total_mass(mf):
    return sum(e["mass"] for e in mf["elements"])


def get_singleton_mass(mf, material_idx):
    """Get the mass assigned to a particular singleton material."""
    target = 1 << material_idx
    for e in mf["elements"]:
        if e["subset"] == target:
            return e["mass"]
    return 0.0


def get_theta_mass(mf):
    """Get the mass assigned to Theta (full ignorance)."""
    for e in mf["elements"]:
        if e["subset"] == DS_THETA:
            return e["mass"]
    return 0.0


# ===========================================================================
# TESTS
# ===========================================================================


class TestSingleSensorEvidence:
    """Test that a single sensor reading produces correct mass assignments."""

    def test_impedance_metal_signature(self):
        """Metal: very low impedance (~1 Ohm), near-zero phase."""
        mf = ds_from_impedance(magnitude=1.0, phase_deg=0.0, confidence=0.8)
        assert total_mass(mf) == pytest.approx(1.0, abs=0.01)
        # Metal should get the highest singleton mass
        metal_mass = get_singleton_mass(mf, MAT_METAL)
        for i in range(MATERIAL_COUNT):
            if i != MAT_METAL:
                assert metal_mass >= get_singleton_mass(mf, i), \
                    f"Metal mass ({metal_mass}) should be >= {MATERIAL_DB[i]['name']} mass ({get_singleton_mass(mf, i)})"

    def test_impedance_glass_signature(self):
        """Glass: extremely high impedance (~10M Ohm), nearly pure capacitive phase."""
        mf = ds_from_impedance(magnitude=10_000_000.0, phase_deg=-88.0, confidence=0.8)
        assert total_mass(mf) == pytest.approx(1.0, abs=0.01)
        glass_mass = get_singleton_mass(mf, MAT_GLASS)
        for i in range(MATERIAL_COUNT):
            if i != MAT_GLASS:
                assert glass_mass >= get_singleton_mass(mf, i)

    def test_impedance_skin_signature(self):
        """Skin: moderate-high impedance (~50k Ohm), moderate capacitive phase."""
        mf = ds_from_impedance(magnitude=50000.0, phase_deg=-35.0, confidence=0.8)
        assert total_mass(mf) == pytest.approx(1.0, abs=0.01)
        skin_mass = get_singleton_mass(mf, MAT_SKIN)
        for i in range(MATERIAL_COUNT):
            if i != MAT_SKIN:
                assert skin_mass >= get_singleton_mass(mf, i)

    def test_acoustic_metal_high_freq(self):
        """Metal: high dominant frequency, slow decay."""
        mf = ds_from_acoustic(dominant_freq=1800.0, decay_ratio=0.35, confidence=0.8)
        assert total_mass(mf) == pytest.approx(1.0, abs=0.01)
        metal_mass = get_singleton_mass(mf, MAT_METAL)
        for i in range(MATERIAL_COUNT):
            if i != MAT_METAL:
                assert metal_mass >= get_singleton_mass(mf, i)

    def test_acoustic_cardboard_low_freq(self):
        """Cardboard: low frequency, very fast decay."""
        mf = ds_from_acoustic(dominant_freq=200.0, decay_ratio=0.03, confidence=0.8)
        assert total_mass(mf) == pytest.approx(1.0, abs=0.01)
        cardboard_mass = get_singleton_mass(mf, MAT_CARDBOARD)
        # Cardboard should be among the top materials
        assert cardboard_mass > 0.01, "Cardboard should have non-trivial mass"

    def test_low_confidence_produces_high_ignorance(self):
        """Very low confidence should assign almost all mass to Theta."""
        mf = ds_from_impedance(magnitude=1.0, phase_deg=0.0, confidence=0.01)
        theta_mass = get_theta_mass(mf)
        assert theta_mass == pytest.approx(1.0, abs=0.01)

    def test_acoustic_low_confidence_produces_ignorance(self):
        """Acoustic sensor with very low confidence should produce full ignorance."""
        mf = ds_from_acoustic(dominant_freq=1000.0, decay_ratio=0.1, confidence=0.02)
        theta_mass = get_theta_mass(mf)
        assert theta_mass == pytest.approx(1.0, abs=0.01)

    def test_curvature_edge_geometry(self):
        """Edge geometry (type 3) should rule out some materials."""
        mf = ds_from_curvature(geometry=3, flatness=0.5, sharp_edge=True)
        assert total_mass(mf) == pytest.approx(1.0, abs=0.01)
        # Edge geometry compatible with: metal (bit4=1), wood (bit3=1), cardboard (bit3=1)
        # Check that the compatible subset mass exists
        has_non_theta = any(e["subset"] != DS_THETA for e in mf["elements"])
        assert has_non_theta, "Edge geometry should produce a non-Theta focal element"

    def test_curvature_all_compatible(self):
        """Geometry 0 (flat) is compatible with all materials -> full ignorance."""
        mf = ds_from_curvature(geometry=0, flatness=0.5, sharp_edge=False)
        assert total_mass(mf) == pytest.approx(1.0, abs=0.01)
        # Flat is compatible with all 6 materials (all have bit 0 set in compatible_geometries)
        # This means num_compatible == MATERIAL_COUNT -> all theta
        theta_mass = get_theta_mass(mf)
        assert theta_mass == pytest.approx(1.0, abs=0.01)


class TestDempsterCombination:
    """Test Dempster's rule of combination."""

    def test_two_agreeing_sensors_metal(self):
        """Two sensors both pointing to metal should produce high metal belief."""
        mf_imp = ds_from_impedance(magnitude=1.0, phase_deg=0.0, confidence=0.8)
        mf_aco = ds_from_acoustic(dominant_freq=1800.0, decay_ratio=0.35, confidence=0.7)
        combined = ds_combine(mf_imp, mf_aco)

        assert total_mass(combined) == pytest.approx(1.0, abs=0.02)

        result = ds_get_result(combined)
        assert result["best_material"] == MAT_METAL
        # Two agreeing sensors should produce high belief
        assert result["best_belief"] > 0.3

    def test_two_agreeing_sensors_glass(self):
        """Two sensors both pointing to glass should produce high glass belief."""
        mf_imp = ds_from_impedance(magnitude=10_000_000.0, phase_deg=-88.0, confidence=0.8)
        mf_aco = ds_from_acoustic(dominant_freq=2800.0, decay_ratio=0.30, confidence=0.7)
        combined = ds_combine(mf_imp, mf_aco)

        result = ds_get_result(combined)
        assert result["best_material"] == MAT_GLASS
        assert result["best_belief"] > 0.3

    def test_two_conflicting_sensors(self):
        """Impedance says metal, acoustic says glass -> high conflict."""
        mf_imp = ds_from_impedance(magnitude=1.0, phase_deg=0.0, confidence=0.9)
        mf_aco = ds_from_acoustic(dominant_freq=2800.0, decay_ratio=0.30, confidence=0.9)
        combined = ds_combine(mf_imp, mf_aco)

        # There should be non-trivial conflict
        assert combined["conflict"] > 0.05, \
            f"Conflict {combined['conflict']} should be significant for contradictory sensors"

    def test_combine_with_vacuous(self):
        """Combining with a vacuous mass (all theta) should not change the result."""
        mf_imp = ds_from_impedance(magnitude=1.0, phase_deg=0.0, confidence=0.8)
        mf_vacuous = {"elements": [{"subset": DS_THETA, "mass": 1.0}], "conflict": 0.0}

        combined = ds_combine(mf_imp, mf_vacuous)
        result = ds_get_result(combined)
        result_orig = ds_get_result(mf_imp)

        # Best material should be the same
        assert result["best_material"] == result_orig["best_material"]
        # Beliefs should be very close
        for i in range(MATERIAL_COUNT):
            assert result["belief"][i] == pytest.approx(result_orig["belief"][i], abs=0.05)

    def test_combine_all_empty_list(self):
        """Combining zero mass functions should return vacuous."""
        result = ds_combine_all([])
        assert len(result["elements"]) == 1
        assert result["elements"][0]["subset"] == DS_THETA
        assert result["elements"][0]["mass"] == pytest.approx(1.0, abs=0.001)

    def test_combine_all_single(self):
        """Combining a single mass function should return it unchanged."""
        mf = ds_from_impedance(magnitude=1.0, phase_deg=0.0, confidence=0.8)
        result = ds_combine_all([mf])
        # Should have same elements
        for e_orig, e_result in zip(mf["elements"], result["elements"]):
            assert e_orig["subset"] == e_result["subset"]
            assert e_orig["mass"] == pytest.approx(e_result["mass"], abs=0.001)

    def test_mass_normalization_after_combine(self):
        """After combination, total mass should be 1.0."""
        mf1 = ds_from_impedance(magnitude=1000.0, phase_deg=-10.0, confidence=0.7)
        mf2 = ds_from_acoustic(dominant_freq=500.0, decay_ratio=0.1, confidence=0.6)
        combined = ds_combine(mf1, mf2)
        assert total_mass(combined) == pytest.approx(1.0, abs=0.02)


class TestAllSensorsAgree:
    """Test fusion when all sensor modalities agree on the same material."""

    @pytest.mark.parametrize("mat_idx,mag,phase,freq,decay,geo", [
        (MAT_METAL,     1.0,        0.0,   1800.0, 0.35, 0),
        (MAT_SKIN,      50000.0,   -35.0,   100.0, 0.01, 1),
        (MAT_PLASTIC,   1000000.0, -85.0,   900.0, 0.12, 0),
        (MAT_WOOD,      500000.0,  -20.0,   350.0, 0.06, 3),
        (MAT_GLASS,     10000000.0,-88.0,  2800.0, 0.30, 1),
        (MAT_CARDBOARD, 200000.0,  -25.0,   200.0, 0.03, 3),
    ])
    def test_all_agree_correct_classification(self, mat_idx, mag, phase, freq, decay, geo):
        """When impedance, acoustic, and curvature all agree, material should be correct."""
        mf_imp = ds_from_impedance(magnitude=mag, phase_deg=phase, confidence=0.8)
        mf_aco = ds_from_acoustic(dominant_freq=freq, decay_ratio=decay, confidence=0.7)
        mf_curv = ds_from_curvature(geometry=geo, flatness=0.7, sharp_edge=False)

        combined = ds_combine_all([mf_imp, mf_aco, mf_curv])
        result = ds_get_result(combined)

        assert result["valid"], f"Result should be valid for {MATERIAL_DB[mat_idx]['name']}"
        assert result["best_material"] == mat_idx, \
            f"Expected {MATERIAL_DB[mat_idx]['name']} but got {MATERIAL_DB[result['best_material']]['name'] if result['best_material'] < MATERIAL_COUNT else 'Unknown'}"


class TestBeliefPlausibility:
    """Test belief and plausibility computation."""

    def test_belief_leq_plausibility(self):
        """For every material, Bel(X) <= Pl(X) must hold."""
        mf_imp = ds_from_impedance(magnitude=1000.0, phase_deg=-30.0, confidence=0.7)
        mf_aco = ds_from_acoustic(dominant_freq=500.0, decay_ratio=0.1, confidence=0.6)
        combined = ds_combine(mf_imp, mf_aco)
        result = ds_get_result(combined)

        for i in range(MATERIAL_COUNT):
            assert result["belief"][i] <= result["plausibility"][i] + 1e-6, \
                f"{MATERIAL_DB[i]['name']}: Bel={result['belief'][i]} > Pl={result['plausibility'][i]}"

    def test_beliefs_sum_leq_one(self):
        """Sum of all singleton beliefs should be <= 1.0."""
        mf = ds_from_impedance(magnitude=1000.0, phase_deg=-30.0, confidence=0.7)
        result = ds_get_result(mf)
        assert sum(result["belief"]) <= 1.0 + 1e-6

    def test_plausibilities_with_ignorance(self):
        """With residual ignorance, plausibility should exceed belief."""
        mf = ds_from_impedance(magnitude=1.0, phase_deg=0.0, confidence=0.5)
        result = ds_get_result(mf)
        # With moderate confidence, there should be residual ignorance
        assert result["mass_theta"] > 0.05
        # Therefore, Pl > Bel for at least some materials
        any_gap = any(
            result["plausibility"][i] > result["belief"][i] + 0.01
            for i in range(MATERIAL_COUNT)
        )
        assert any_gap, "With ignorance, at least one material should have Pl > Bel"


class TestEdgeCases:
    """Test edge cases and boundary conditions."""

    def test_zero_confidence_all_sensors(self):
        """All sensors report zero confidence -> graceful vacuous result."""
        mf_imp = ds_from_impedance(magnitude=1.0, phase_deg=0.0, confidence=0.0)
        mf_aco = ds_from_acoustic(dominant_freq=500.0, decay_ratio=0.1, confidence=0.0)

        combined = ds_combine(mf_imp, mf_aco)
        result = ds_get_result(combined)

        # With zero confidence, everything goes to theta
        assert result["mass_theta"] == pytest.approx(1.0, abs=0.01)
        # Result may or may not be valid, but should not crash
        assert result["best_belief"] <= 0.15

    def test_zero_magnitude_impedance(self):
        """Zero impedance magnitude should be handled (clamped to 0.01)."""
        mf = ds_from_impedance(magnitude=0.0, phase_deg=0.0, confidence=0.8)
        assert total_mass(mf) == pytest.approx(1.0, abs=0.01)

    def test_extreme_impedance(self):
        """Extremely high impedance should still produce valid masses."""
        mf = ds_from_impedance(magnitude=1e12, phase_deg=-90.0, confidence=0.8)
        assert total_mass(mf) == pytest.approx(1.0, abs=0.01)

    def test_near_total_conflict(self):
        """When conflict approaches 1.0, should return vacuous or near-vacuous."""
        # Create two mass functions that are nearly completely contradictory
        # One says "definitely metal" (only metal singleton), other says "definitely glass"
        mf1 = {"elements": [
            {"subset": 1 << MAT_METAL, "mass": 0.99},
            {"subset": DS_THETA, "mass": 0.01},
        ], "conflict": 0.0}
        mf2 = {"elements": [
            {"subset": 1 << MAT_GLASS, "mass": 0.99},
            {"subset": DS_THETA, "mass": 0.01},
        ], "conflict": 0.0}

        combined = ds_combine(mf1, mf2)
        # Should have very high conflict
        assert combined["conflict"] > 0.9
        # Should still produce valid total mass
        assert total_mass(combined) == pytest.approx(1.0, abs=0.05)

    def test_identical_mass_functions(self):
        """Combining identical mass functions should reinforce the same material."""
        mf = ds_from_impedance(magnitude=1.0, phase_deg=0.0, confidence=0.7)
        result_single = ds_get_result(mf)
        combined = ds_combine(mf, mf)
        result_double = ds_get_result(combined)

        # Same best material
        assert result_double["best_material"] == result_single["best_material"]
        # Belief should increase (or at least not decrease)
        assert result_double["best_belief"] >= result_single["best_belief"] - 0.01


class TestKnownMaterialSignatures:
    """Test classification with known material signatures from MATERIAL_DB."""

    @pytest.mark.parametrize("mat_idx", range(MATERIAL_COUNT))
    def test_impedance_matches_database(self, mat_idx):
        """Each material's own impedance signature should classify correctly."""
        mag = MATERIAL_DB[mat_idx]["impedance_magnitude"]
        phase = MATERIAL_DB[mat_idx]["impedance_phase_deg"]
        mf = ds_from_impedance(magnitude=mag, phase_deg=phase, confidence=0.9)
        result = ds_get_result(mf)

        assert result["best_material"] == mat_idx, \
            f"Expected {MATERIAL_DB[mat_idx]['name']} but got " \
            f"{MATERIAL_DB[result['best_material']]['name'] if result['best_material'] < MATERIAL_COUNT else 'Unknown'}"

    @pytest.mark.parametrize("mat_idx", range(MATERIAL_COUNT))
    def test_acoustic_matches_database(self, mat_idx):
        """Each material's own acoustic signature should classify correctly."""
        m = MATERIAL_MODELS[mat_idx]
        mf = ds_from_acoustic(
            dominant_freq=m["aco_freq_mean"],
            decay_ratio=m["aco_decay_mean"],
            confidence=0.9,
        )
        result = ds_get_result(mf)

        assert result["best_material"] == mat_idx, \
            f"Expected {MATERIAL_DB[mat_idx]['name']} but got " \
            f"{MATERIAL_DB[result['best_material']]['name'] if result['best_material'] < MATERIAL_COUNT else 'Unknown'}"
