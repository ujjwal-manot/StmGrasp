"""
Tests for the Grasp Planner state machine and strategy selection.
Ported from esp32_brain/grasp_planner.cpp and grasp_planner.h

The grasp FSM has 12 states:
  IDLE -> DETECTED -> SCANNING -> ANALYZING -> TAP_TESTING ->
  CLASSIFYING -> PLANNING -> APPROACHING -> GRIPPING -> HOLDING ->
  RELEASING -> ERROR

Strategy repertoire: POWER, PRECISION, WRAP, EDGE
"""

import pytest

# ---------------------------------------------------------------------------
# Constants ported from config.h and grasp_planner.cpp
# ---------------------------------------------------------------------------

MATERIAL_COUNT = 6
MAT_METAL = 0
MAT_SKIN = 1
MAT_PLASTIC = 2
MAT_WOOD = 3
MAT_GLASS = 4
MAT_CARDBOARD = 5
MAT_UNKNOWN = 255

# Grasp states
STATE_IDLE = 0
STATE_DETECTED = 1
STATE_SCANNING = 2
STATE_ANALYZING = 3
STATE_TAP_TESTING = 4
STATE_CLASSIFYING = 5
STATE_PLANNING = 6
STATE_APPROACHING = 7
STATE_GRIPPING = 8
STATE_HOLDING = 9
STATE_RELEASING = 10
STATE_ERROR = 11

STATE_NAMES = [
    "IDLE", "DETECTED", "SCANNING", "ANALYZING", "TAP_TESTING",
    "CLASSIFYING", "PLANNING", "APPROACHING", "GRIPPING",
    "HOLDING", "RELEASING", "ERROR"
]

# Grasp strategies
STRATEGY_POWER = 0
STRATEGY_PRECISION = 1
STRATEGY_WRAP = 2
STRATEGY_EDGE = 3
STRATEGY_COUNT = 4

STRATEGY_NAMES = ["POWER", "PRECISION", "WRAP", "EDGE"]

# Timeouts
STATE_TIMEOUT_SCANNING_MS = 3000
STATE_TIMEOUT_ANALYZING_MS = 2000
STATE_TIMEOUT_TAP_MS = 4000
STATE_TIMEOUT_CLASSIFYING_MS = 1000
STATE_TIMEOUT_PLANNING_MS = 1000
STATE_TIMEOUT_APPROACHING_MS = 5000
STATE_TIMEOUT_GRIPPING_MS = 3000

GRASP_QUALITY_MIN = 0.3
GRASP_FORCE_SAFETY_MARGIN = 0.85
SP_PROCEED_THRESHOLD = 0.55

# Material database from config.h
MATERIAL_DB = [
    {"name": "Metal",     "max_grip_force_N": 8.0, "slip_risk": 0.2},
    {"name": "Skin",      "max_grip_force_N": 1.5, "slip_risk": 0.3},
    {"name": "Plastic",   "max_grip_force_N": 5.0, "slip_risk": 0.6},
    {"name": "Wood",      "max_grip_force_N": 6.0, "slip_risk": 0.3},
    {"name": "Glass",     "max_grip_force_N": 4.0, "slip_risk": 0.8},
    {"name": "Cardboard", "max_grip_force_N": 2.0, "slip_risk": 0.4},
]


# ---------------------------------------------------------------------------
# Python port of the strategy selection logic
# ---------------------------------------------------------------------------

def select_strategy(material, geometry, flatness):
    """
    Port of selectStrategy() from grasp_planner.cpp.
    Selects best grasp strategy based on material, geometry, and flatness.
    """
    if material >= MATERIAL_COUNT:
        return STRATEGY_POWER

    fragility = 1.0 - (MATERIAL_DB[material]["max_grip_force_N"] / 10.0)
    fragility = min(max(fragility, 0.0), 1.0)
    slip_risk = MATERIAL_DB[material]["slip_risk"]

    # Edge geometry always gets EDGE strategy
    if geometry == 3:
        return STRATEGY_EDGE

    # Fragile materials get PRECISION
    if fragility > 0.6:
        return STRATEGY_PRECISION

    # Curved/cylindrical geometries
    if geometry in (1, 2, 4):  # convex, concave, cylinder
        return STRATEGY_WRAP if slip_risk > 0.5 else STRATEGY_POWER

    # Flat, robust objects
    if flatness > 0.8 and fragility < 0.3:
        return STRATEGY_POWER

    # Default
    return STRATEGY_PRECISION


# ---------------------------------------------------------------------------
# Grasp FSM simulation
# ---------------------------------------------------------------------------

class GraspFSM:
    """
    Python port of the grasp state machine from grasp_planner.cpp.
    Tracks state, elapsed time, e-stop, and state requests.
    """

    def __init__(self):
        self.state = STATE_IDLE
        self.state_enter_ms = 0
        self.current_time_ms = 0
        self.estop = False
        self.requested_state = STATE_IDLE
        self.state_request_pending = False

    def trigger_estop(self):
        self.estop = True

    def request_state_transition(self, target):
        self.requested_state = target
        self.state_request_pending = True

    def _enter_state(self, s):
        self.state = s
        self.state_enter_ms = self.current_time_ms

    def advance_time(self, ms):
        self.current_time_ms += ms

    def run(self, depth_valid=False, depth_min_mm=9999,
            curv_valid=False, imp_valid=False,
            force_valid=False, force_total_N=0.0,
            force_target_N=3.0, force_slip_detected=False,
            plan_valid=False, plan_success_prob=0.0,
            plan_quality_score=0.0, grasp_quality=0.0):
        """
        Run one step of the state machine.
        Simplified version of runStateMachine() from grasp_planner.cpp.
        Returns the new state.
        """
        # E-stop handling
        if self.estop:
            self.estop = False
            self._enter_state(STATE_RELEASING)
            return self.state

        # State request handling
        if self.state_request_pending:
            self.state_request_pending = False
            req = self.requested_state
            if req == STATE_IDLE or req == STATE_RELEASING:
                self._enter_state(req)
                return self.state
            if req == STATE_DETECTED and self.state == STATE_IDLE:
                self._enter_state(req)
                return self.state

        elapsed = self.current_time_ms - self.state_enter_ms

        if self.state == STATE_IDLE:
            if depth_valid and depth_min_mm < 300:
                self._enter_state(STATE_DETECTED)

        elif self.state == STATE_DETECTED:
            if depth_valid and depth_min_mm < 200:
                self._enter_state(STATE_SCANNING)
            elif depth_valid and depth_min_mm > 500:
                self._enter_state(STATE_IDLE)
            if elapsed > 5000:
                self._enter_state(STATE_IDLE)

        elif self.state == STATE_SCANNING:
            if curv_valid and depth_valid:
                self._enter_state(STATE_ANALYZING)
            if elapsed > STATE_TIMEOUT_SCANNING_MS:
                self._enter_state(STATE_ANALYZING)

        elif self.state == STATE_ANALYZING:
            if imp_valid:
                self._enter_state(STATE_TAP_TESTING)
            if elapsed > STATE_TIMEOUT_ANALYZING_MS:
                self._enter_state(STATE_TAP_TESTING)

        elif self.state == STATE_TAP_TESTING:
            if elapsed > STATE_TIMEOUT_TAP_MS:
                self._enter_state(STATE_CLASSIFYING)

        elif self.state == STATE_CLASSIFYING:
            # Immediately transitions to PLANNING
            self._enter_state(STATE_PLANNING)

        elif self.state == STATE_PLANNING:
            if plan_valid:
                if plan_success_prob >= SP_PROCEED_THRESHOLD:
                    self._enter_state(STATE_APPROACHING)
                else:
                    self._enter_state(STATE_ERROR)
            if elapsed > STATE_TIMEOUT_PLANNING_MS:
                if plan_quality_score > 0.2:
                    self._enter_state(STATE_APPROACHING)
                else:
                    self._enter_state(STATE_ERROR)

        elif self.state == STATE_APPROACHING:
            if force_valid and force_total_N > 0.2:
                self._enter_state(STATE_GRIPPING)
            if elapsed > STATE_TIMEOUT_APPROACHING_MS:
                self._enter_state(STATE_ERROR)

        elif self.state == STATE_GRIPPING:
            if force_valid and force_total_N >= force_target_N * 0.9:
                self._enter_state(STATE_HOLDING)
            if elapsed > STATE_TIMEOUT_GRIPPING_MS:
                if force_valid and force_total_N > 0.5:
                    self._enter_state(STATE_HOLDING)
                else:
                    self._enter_state(STATE_ERROR)

        elif self.state == STATE_HOLDING:
            if force_valid:
                if force_total_N < 0.1:
                    self._enter_state(STATE_ERROR)
                if force_slip_detected and grasp_quality < 0.15:
                    self._enter_state(STATE_ERROR)

        elif self.state == STATE_RELEASING:
            if force_valid and force_total_N < 0.05:
                self._enter_state(STATE_IDLE)
            if elapsed > 3000:
                self._enter_state(STATE_IDLE)

        elif self.state == STATE_ERROR:
            if elapsed > 2000:
                self._enter_state(STATE_IDLE)

        return self.state


# ===========================================================================
# TESTS
# ===========================================================================


class TestStateTransitionBasic:
    """Test fundamental state transitions."""

    def test_idle_to_detected_on_close_object(self):
        """IDLE -> DETECTED when depth sensor detects object < 300mm."""
        fsm = GraspFSM()
        assert fsm.state == STATE_IDLE

        fsm.run(depth_valid=True, depth_min_mm=250)
        assert fsm.state == STATE_DETECTED

    def test_idle_stays_when_no_object(self):
        """Should stay IDLE when no object detected."""
        fsm = GraspFSM()
        fsm.run(depth_valid=True, depth_min_mm=1000)
        assert fsm.state == STATE_IDLE

    def test_idle_stays_when_depth_invalid(self):
        """Should stay IDLE when depth sensor is invalid."""
        fsm = GraspFSM()
        fsm.run(depth_valid=False, depth_min_mm=100)
        assert fsm.state == STATE_IDLE

    def test_detected_to_scanning(self):
        """DETECTED -> SCANNING when object moves closer (< 200mm)."""
        fsm = GraspFSM()
        fsm.run(depth_valid=True, depth_min_mm=250)  # -> DETECTED
        assert fsm.state == STATE_DETECTED

        fsm.run(depth_valid=True, depth_min_mm=150)  # -> SCANNING
        assert fsm.state == STATE_SCANNING

    def test_detected_back_to_idle_far_object(self):
        """DETECTED -> IDLE when object moves far away (> 500mm)."""
        fsm = GraspFSM()
        fsm.run(depth_valid=True, depth_min_mm=250)  # -> DETECTED
        assert fsm.state == STATE_DETECTED

        fsm.run(depth_valid=True, depth_min_mm=600)  # -> IDLE
        assert fsm.state == STATE_IDLE

    def test_scanning_to_analyzing(self):
        """SCANNING -> ANALYZING when curvature and depth both valid."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_SCANNING)
        fsm.run(curv_valid=True, depth_valid=True)
        assert fsm.state == STATE_ANALYZING

    def test_analyzing_to_tap_testing(self):
        """ANALYZING -> TAP_TESTING when impedance valid."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_ANALYZING)
        fsm.run(imp_valid=True)
        assert fsm.state == STATE_TAP_TESTING

    def test_classifying_to_planning(self):
        """CLASSIFYING -> PLANNING immediately."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_CLASSIFYING)
        fsm.run()
        assert fsm.state == STATE_PLANNING

    def test_approaching_to_gripping(self):
        """APPROACHING -> GRIPPING when force detected."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_APPROACHING)
        fsm.run(force_valid=True, force_total_N=0.5)
        assert fsm.state == STATE_GRIPPING

    def test_gripping_to_holding(self):
        """GRIPPING -> HOLDING when force reaches target."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_GRIPPING)
        fsm.run(force_valid=True, force_total_N=3.0, force_target_N=3.0)
        assert fsm.state == STATE_HOLDING

    def test_releasing_to_idle(self):
        """RELEASING -> IDLE when force drops below 0.05N."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_RELEASING)
        fsm.run(force_valid=True, force_total_N=0.01)
        assert fsm.state == STATE_IDLE


class TestFullCycle:
    """Test a complete grasp cycle from IDLE back to IDLE."""

    def test_full_successful_cycle(self):
        """Full cycle: IDLE -> DETECTED -> SCANNING -> ANALYZING -> TAP_TESTING ->
        CLASSIFYING -> PLANNING -> APPROACHING -> GRIPPING -> HOLDING -> RELEASING -> IDLE"""
        fsm = GraspFSM()

        # IDLE -> DETECTED
        fsm.run(depth_valid=True, depth_min_mm=250)
        assert fsm.state == STATE_DETECTED

        # DETECTED -> SCANNING
        fsm.run(depth_valid=True, depth_min_mm=150)
        assert fsm.state == STATE_SCANNING

        # SCANNING -> ANALYZING (both sensors valid)
        fsm.run(curv_valid=True, depth_valid=True)
        assert fsm.state == STATE_ANALYZING

        # ANALYZING -> TAP_TESTING (impedance valid)
        fsm.run(imp_valid=True)
        assert fsm.state == STATE_TAP_TESTING

        # TAP_TESTING -> CLASSIFYING (timeout)
        fsm.advance_time(STATE_TIMEOUT_TAP_MS + 1)
        fsm.run()
        assert fsm.state == STATE_CLASSIFYING

        # CLASSIFYING -> PLANNING (immediate)
        fsm.run()
        assert fsm.state == STATE_PLANNING

        # PLANNING -> APPROACHING (plan valid + high success prob)
        fsm.run(plan_valid=True, plan_success_prob=0.7)
        assert fsm.state == STATE_APPROACHING

        # APPROACHING -> GRIPPING (force detected)
        fsm.run(force_valid=True, force_total_N=0.5)
        assert fsm.state == STATE_GRIPPING

        # GRIPPING -> HOLDING (force at target)
        fsm.run(force_valid=True, force_total_N=3.0, force_target_N=3.0)
        assert fsm.state == STATE_HOLDING

        # Simulate holding for a while (stable)
        fsm.advance_time(1000)
        fsm.run(force_valid=True, force_total_N=3.0, grasp_quality=0.8)
        assert fsm.state == STATE_HOLDING

        # Trigger release via state request
        fsm.request_state_transition(STATE_RELEASING)
        fsm.run()
        assert fsm.state == STATE_RELEASING

        # RELEASING -> IDLE (force released)
        fsm.run(force_valid=True, force_total_N=0.01)
        assert fsm.state == STATE_IDLE


class TestTimeouts:
    """Test timeout handling for each state."""

    def test_detected_timeout(self):
        """DETECTED should time out to IDLE after 5000ms."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_DETECTED)
        fsm.advance_time(5001)
        fsm.run(depth_valid=True, depth_min_mm=400)  # object at medium range, no transition trigger
        assert fsm.state == STATE_IDLE

    def test_scanning_timeout(self):
        """SCANNING should time out to ANALYZING."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_SCANNING)
        fsm.advance_time(STATE_TIMEOUT_SCANNING_MS + 1)
        fsm.run()
        assert fsm.state == STATE_ANALYZING

    def test_analyzing_timeout(self):
        """ANALYZING should time out to TAP_TESTING."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_ANALYZING)
        fsm.advance_time(STATE_TIMEOUT_ANALYZING_MS + 1)
        fsm.run()
        assert fsm.state == STATE_TAP_TESTING

    def test_tap_testing_timeout(self):
        """TAP_TESTING should time out to CLASSIFYING."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_TAP_TESTING)
        fsm.advance_time(STATE_TIMEOUT_TAP_MS + 1)
        fsm.run()
        assert fsm.state == STATE_CLASSIFYING

    def test_planning_timeout_good_quality(self):
        """PLANNING times out -> APPROACHING if quality > 0.2."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_PLANNING)
        fsm.advance_time(STATE_TIMEOUT_PLANNING_MS + 1)
        fsm.run(plan_quality_score=0.5)
        assert fsm.state == STATE_APPROACHING

    def test_planning_timeout_bad_quality(self):
        """PLANNING times out -> ERROR if quality <= 0.2."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_PLANNING)
        fsm.advance_time(STATE_TIMEOUT_PLANNING_MS + 1)
        fsm.run(plan_quality_score=0.1)
        assert fsm.state == STATE_ERROR

    def test_approaching_timeout(self):
        """APPROACHING should time out to ERROR."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_APPROACHING)
        fsm.advance_time(STATE_TIMEOUT_APPROACHING_MS + 1)
        fsm.run()
        assert fsm.state == STATE_ERROR

    def test_gripping_timeout_with_force(self):
        """GRIPPING timeout with some force -> HOLDING."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_GRIPPING)
        fsm.advance_time(STATE_TIMEOUT_GRIPPING_MS + 1)
        fsm.run(force_valid=True, force_total_N=1.0)
        assert fsm.state == STATE_HOLDING

    def test_gripping_timeout_no_force(self):
        """GRIPPING timeout with no force -> ERROR."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_GRIPPING)
        fsm.advance_time(STATE_TIMEOUT_GRIPPING_MS + 1)
        fsm.run(force_valid=True, force_total_N=0.1)
        assert fsm.state == STATE_ERROR

    def test_releasing_timeout(self):
        """RELEASING should time out to IDLE after 3000ms."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_RELEASING)
        fsm.advance_time(3001)
        fsm.run()
        assert fsm.state == STATE_IDLE

    def test_error_timeout(self):
        """ERROR should time out to IDLE after 2000ms."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_ERROR)
        fsm.advance_time(2001)
        fsm.run()
        assert fsm.state == STATE_IDLE


class TestEstop:
    """Test emergency stop from every state."""

    @pytest.mark.parametrize("start_state", range(12))
    def test_estop_from_any_state(self, start_state):
        """E-STOP from any state should go to RELEASING."""
        fsm = GraspFSM()
        fsm._enter_state(start_state)
        fsm.trigger_estop()
        fsm.run()
        assert fsm.state == STATE_RELEASING, \
            f"E-STOP from {STATE_NAMES[start_state]} should go to RELEASING, got {STATE_NAMES[fsm.state]}"

    def test_estop_then_idle(self):
        """After E-STOP -> RELEASING, should eventually return to IDLE."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_HOLDING)
        fsm.trigger_estop()
        fsm.run()
        assert fsm.state == STATE_RELEASING

        # Release completes
        fsm.run(force_valid=True, force_total_N=0.01)
        assert fsm.state == STATE_IDLE


class TestHoldingFailures:
    """Test failure conditions during HOLDING state."""

    def test_holding_force_lost(self):
        """HOLDING -> ERROR when force drops to near zero."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_HOLDING)
        fsm.run(force_valid=True, force_total_N=0.05, grasp_quality=0.8)
        assert fsm.state == STATE_ERROR

    def test_holding_slip_with_low_quality(self):
        """HOLDING -> ERROR when slip detected and grasp quality is very low."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_HOLDING)
        fsm.run(force_valid=True, force_total_N=2.0,
                force_slip_detected=True, grasp_quality=0.1)
        assert fsm.state == STATE_ERROR

    def test_holding_slip_with_good_quality(self):
        """HOLDING stays if slip detected but grasp quality is acceptable."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_HOLDING)
        fsm.run(force_valid=True, force_total_N=2.0,
                force_slip_detected=True, grasp_quality=0.5)
        assert fsm.state == STATE_HOLDING

    def test_holding_stable(self):
        """HOLDING remains stable with good force and no slip."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_HOLDING)
        for _ in range(10):
            fsm.advance_time(100)
            fsm.run(force_valid=True, force_total_N=3.0, grasp_quality=0.8)
        assert fsm.state == STATE_HOLDING


class TestPlanningDecision:
    """Test planning state decision logic."""

    def test_planning_high_success_prob(self):
        """PLANNING -> APPROACHING when success_prob >= threshold."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_PLANNING)
        fsm.run(plan_valid=True, plan_success_prob=0.7)
        assert fsm.state == STATE_APPROACHING

    def test_planning_low_success_prob(self):
        """PLANNING -> ERROR when success_prob < threshold."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_PLANNING)
        fsm.run(plan_valid=True, plan_success_prob=0.3)
        assert fsm.state == STATE_ERROR

    def test_planning_exactly_at_threshold(self):
        """PLANNING -> APPROACHING when success_prob exactly at threshold."""
        fsm = GraspFSM()
        fsm._enter_state(STATE_PLANNING)
        fsm.run(plan_valid=True, plan_success_prob=SP_PROCEED_THRESHOLD)
        assert fsm.state == STATE_APPROACHING


class TestStrategySelection:
    """Test correct strategy selection for each material type."""

    def test_metal_flat_power(self):
        """Metal on flat surface -> POWER (robust, low fragility)."""
        strat = select_strategy(MAT_METAL, geometry=0, flatness=0.9)
        assert strat == STRATEGY_POWER

    def test_skin_flat_precision(self):
        """Skin on flat surface -> PRECISION (fragile: max_grip=1.5N, fragility=0.85)."""
        strat = select_strategy(MAT_SKIN, geometry=0, flatness=0.9)
        assert strat == STRATEGY_PRECISION

    def test_glass_flat_precision(self):
        """Glass on flat surface -> PRECISION (fragile: max_grip=4.0N, fragility=0.6)."""
        strat = select_strategy(MAT_GLASS, geometry=0, flatness=0.5)
        # Glass fragility = 1 - (4.0/10.0) = 0.6, which is exactly at the threshold.
        # In the C code: if (fragility > 0.6f) -> not triggered at exactly 0.6
        # So glass with geometry=0 and flatness=0.5 -> PRECISION (default)
        assert strat == STRATEGY_PRECISION

    def test_cardboard_flat_precision(self):
        """Cardboard on flat surface -> PRECISION (fragile: max_grip=2.0N, fragility=0.8)."""
        strat = select_strategy(MAT_CARDBOARD, geometry=0, flatness=0.9)
        assert strat == STRATEGY_PRECISION

    def test_any_material_edge_geometry(self):
        """Any material with edge geometry -> EDGE strategy."""
        for mat in range(MATERIAL_COUNT):
            strat = select_strategy(mat, geometry=3, flatness=0.5)
            assert strat == STRATEGY_EDGE, \
                f"{MATERIAL_DB[mat]['name']} with edge geometry should get EDGE, got {STRATEGY_NAMES[strat]}"

    def test_plastic_cylinder_wrap(self):
        """Plastic on cylinder -> WRAP (curved + high slip_risk=0.6)."""
        strat = select_strategy(MAT_PLASTIC, geometry=4, flatness=0.5)
        assert strat == STRATEGY_WRAP

    def test_metal_cylinder_power(self):
        """Metal on cylinder -> POWER (curved but low slip_risk=0.2)."""
        strat = select_strategy(MAT_METAL, geometry=4, flatness=0.5)
        assert strat == STRATEGY_POWER

    def test_wood_flat_power(self):
        """Wood on very flat surface -> POWER (robust: max_grip=6.0N, fragility=0.4, but flatness>0.8 and fragility<0.3 NOT met => PRECISION)."""
        # Wood fragility = 1 - (6.0/10.0) = 0.4 -> NOT > 0.6, so not PRECISION from fragility check
        # geometry=0 -> not edge, not curved
        # flatness=0.9 > 0.8 but fragility=0.4 >= 0.3, so POWER condition fails
        # Default: PRECISION
        strat = select_strategy(MAT_WOOD, geometry=0, flatness=0.9)
        assert strat == STRATEGY_PRECISION

    def test_metal_flat_high_flatness_power(self):
        """Metal, flat, high flatness -> POWER (flatness > 0.8 and fragility < 0.3)."""
        # Metal fragility = 1 - (8.0/10.0) = 0.2 < 0.3, flatness 0.9 > 0.8
        strat = select_strategy(MAT_METAL, geometry=0, flatness=0.9)
        assert strat == STRATEGY_POWER

    def test_unknown_material_power(self):
        """Unknown material (>= MATERIAL_COUNT) -> POWER (default)."""
        strat = select_strategy(MAT_UNKNOWN, geometry=0, flatness=0.5)
        assert strat == STRATEGY_POWER

    def test_glass_convex_wrap(self):
        """Glass on convex surface -> WRAP (curved + high slip_risk=0.8)."""
        # Glass fragility = 0.6, NOT > 0.6, so doesn't hit fragility check
        # geometry=1 (convex) -> curved check: slip_risk=0.8 > 0.5 -> WRAP
        strat = select_strategy(MAT_GLASS, geometry=1, flatness=0.5)
        assert strat == STRATEGY_WRAP

    def test_skin_convex_power(self):
        """Skin on convex -> PRECISION, because fragility (0.85) > 0.6 triggers first."""
        strat = select_strategy(MAT_SKIN, geometry=1, flatness=0.5)
        assert strat == STRATEGY_PRECISION
