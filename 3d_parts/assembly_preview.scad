// ============================================================================
// assembly_preview.scad - Full Assembly Preview
// Part of HYDRA GRASP system for STEVAL-ROBKIT1
// All dimensions in mm
//
// This file imports all individual parts and positions them in their
// assembled configuration. It also shows simplified servo representations
// and coordinate reference arrows.
//
// NOTE: This file is for visualization only - do not print this file.
// Print each individual part file separately.
//
// USAGE: Open in OpenSCAD and press F5 for preview, F6 for full render.
// ============================================================================

$fn = 60;

// ========================= CONFIGURATION ====================================

// Toggle visibility of each part
show_base         = true;
show_left_finger  = true;
show_right_finger = true;
show_ir_ring      = true;
show_contact_pads = true;
show_servos       = true;   // simplified servo boxes
show_axes         = true;   // coordinate reference arrows

// Finger open angle (for visualization - adjust to see fingers open/close)
finger_open_angle = 30;   // degrees each finger opens

// ========================= PART MODULES =====================================
// We use `use` to import modules from other files, but since the individual
// files render their part at the top level, we need to use `include` or
// replicate. For cleaner assembly, we replicate simplified versions or
// use `use` with module calls.

// Include the individual part files
// Note: OpenSCAD `use` imports modules but doesn't execute top-level code
use <gripper_finger.scad>
use <gripper_base.scad>
use <ir_led_ring.scad>
use <spring_contact_pad.scad>
use <sensor_backpack.scad>

// ========================= HELPER MODULES ===================================

// Simplified SG90 servo representation (transparent box)
module servo_sg90_simplified() {
    color("DimGray", 0.6) {
        // Main body
        cube([23, 12.2, 29]);

        // Mounting tabs
        translate([-2.5, 0, 15.5 - 0.5])
            cube([28, 12.2, 1]);

        // Output shaft
        translate([6, 12.2 / 2, 29])
            cylinder(d = 4.5, h = 4);
    }
}

// Coordinate reference arrows
module reference_axes(length = 30) {
    // X axis - Red
    color("Red") {
        cylinder(d = 1, h = length);
        translate([0, 0, length])
            cylinder(d1 = 3, d2 = 0, h = 5);
    }
    color("Red")
        translate([2, 0, length + 6])
            linear_extrude(1)
                text("X", size = 5, halign = "center", valign = "center");

    // Y axis - Green
    color("Green") {
        rotate([0, -90, 0]) {
            cylinder(d = 1, h = length);
            translate([0, 0, length])
                cylinder(d1 = 3, d2 = 0, h = 5);
        }
    }
    color("Green")
        translate([-length - 8, 0, 2])
            rotate([0, 0, 90])
                linear_extrude(1)
                    text("Y", size = 5, halign = "center", valign = "center");

    // Z axis - Blue
    color("Blue") {
        rotate([0, 90, 90]) {
            cylinder(d = 1, h = length);
            translate([0, 0, length])
                cylinder(d1 = 3, d2 = 0, h = 5);
        }
    }
    color("Blue")
        translate([0, length + 7, 2])
            linear_extrude(1)
                text("Z", size = 5, halign = "center", valign = "center");
}

// ========================= ASSEMBLY =========================================

module full_assembly() {

    // --- Gripper Base ---
    if (show_base) {
        color("SteelBlue", 0.85)
            translate([-40, -30, 0])  // center the base
                gripper_base();
    }

    // --- Simplified Servos in their pockets ---
    if (show_servos) {
        // Left finger servo
        translate([-40 + 12.25, -30 + 10.25, 3])
            servo_sg90_simplified();

        // Right finger servo
        translate([-40 + 12.25 + 23.5 + 6, -30 + 10.25, 3])
            servo_sg90_simplified();

        // Wrist servo (rotated 90 deg at back)
        if (true) {
            translate([0, 20 - 12.2/2, 5])
                rotate([0, 0, 90])
                    translate([-23/2, -12.2/2, 0])
                        servo_sg90_simplified();
        }
    }

    // --- Left Finger ---
    if (show_left_finger) {
        // Position at left servo shaft output, angled open
        translate([-40 + 12.25 + 6, -30 + 10.25 + 29.5/2, 30 + 4])
            rotate([0, 0, -finger_open_angle])
                rotate([0, -90, 0])
                    translate([-8, -10, 0])  // offset to servo horn mount
                        color("Coral", 0.85)
                            gripper_finger();
    }

    // --- Right Finger (mirrored) ---
    if (show_right_finger) {
        // Position at right servo shaft output, mirrored and angled
        translate([-40 + 12.25 + 23.5 + 6 + 6, -30 + 10.25 + 29.5/2, 30 + 4])
            rotate([0, 0, finger_open_angle])
                rotate([0, -90, 0])
                    mirror([0, 1, 0])
                        translate([-8, -10, 0])
                            color("MediumSeaGreen", 0.85)
                                gripper_finger();
    }

    // --- Spring Contact Pads (inside fingers) ---
    if (show_contact_pads) {
        // Left finger contact pad (approximate position at fingertip)
        translate([-40 + 12.25 + 6 - 50, -30 + 10.25 + 29.5/2, 30 + 4])
            rotate([0, 0, -finger_open_angle])
                color("Gold", 0.9)
                    spring_contact_pad();

        // Right finger contact pad
        translate([-40 + 12.25 + 23.5 + 6 + 6 + 50, -30 + 10.25 + 29.5/2, 30 + 4])
            rotate([0, 0, finger_open_angle])
                color("Gold", 0.9)
                    spring_contact_pad();
    }

    // --- IR LED Ring (at front, below fingers) ---
    if (show_ir_ring) {
        translate([0, -40, 15])
            rotate([90, 0, 0])
                color("Purple", 0.8)
                    ir_led_ring();
    }

    // --- Sensor Backpack (behind the base, on robot body) ---
    color("DarkSlateGray", 0.7)
        translate([-40, 25, 0])
            sensor_backpack();

    // --- Reference Coordinate Arrows ---
    if (show_axes) {
        translate([-55, -45, 0])
            reference_axes(25);
    }
}

// Render the full assembly
full_assembly();
