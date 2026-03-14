// ============================================================================
// gripper_finger.scad - Gripper Finger with Integrated Sensor Mounts
// Part of HYDRA GRASP system for STEVAL-ROBKIT1
// All dimensions in mm
//
// PRINT ORIENTATION: Lay flat on the outer face (the face opposite the
// inner/sensor face). This avoids supports for the FSR pocket, TCRT slots,
// and cable channel. The servo horn mount holes print vertically.
//
// Print Settings: 0.2mm layer height, 3 perimeters, 20% infill, PLA/PETG
// ============================================================================

$fn = 60;

// ========================= PARAMETERS =======================================

// Finger body
finger_length   = 70;
finger_width    = 20;
finger_thickness = 8;
tip_width       = 12;
taper_length    = 20;   // length over which the finger tapers

// Fillet radius for external edges
fillet_r = 0.8;

// FSR 402 sensor pocket (on inner face at tip)
fsr_diameter    = 13;   // 12.7mm sensor + clearance
fsr_depth       = 0.5;

// Copper electrode channels (on tip contact face)
electrode_width   = 7;    // 6mm tape + 1mm clearance
electrode_depth   = 0.5;
electrode_length  = 18;   // length of groove on the tip
electrode_spacing = 3;    // gap between the two grooves

// Spring socket (at tip for spring-loaded contact pad)
spring_socket_dia   = 6;
spring_socket_depth = 8;

// TCRT5000 IR sensor slots (along inner face)
tcrt_count   = 2;
tcrt_width   = 11;   // 10.2mm + clearance
tcrt_height  = 6;    // 5.8mm + clearance (slot height along finger thickness)
tcrt_depth   = 8;    // 7mm sensor depth + clearance
tcrt_spacing = 15;   // center-to-center spacing
tcrt_angle   = 10;   // degrees angled inward
tcrt_start   = 20;   // distance from tip center to first TCRT slot center

// Piezo disc mount (on inner face near base)
has_piezo      = true;   // set false for right finger
piezo_diameter = 28;     // 27mm disc + clearance
piezo_depth    = 1;      // 0.5mm disc + adhesive clearance
piezo_offset   = 12;     // distance from base end to piezo center

// Cable channel (along the back/outer face)
cable_width  = 3;
cable_depth  = 2;

// Servo horn mount (at base)
// SG90 horn: 2 screw holes at 7mm radius from center, M2 screws
horn_hole_radius    = 7;    // distance from center to each screw hole
horn_hole_diameter  = 2.2;  // M2 + clearance
horn_center_hole    = 4.5;  // center hole for servo shaft boss

// Finger identification
finger_id = "L";   // "L" or "R"
text_size = 6;
text_depth = 0.6;

// ========================= DERIVED VALUES ===================================

straight_length = finger_length - taper_length;

// ========================= MODULES ==========================================

// Basic finger body: a straight section + tapered tip, built with hull()
module finger_body() {
    // Straight section from base to start of taper
    translate([0, 0, 0])
        cube([straight_length, finger_width, finger_thickness]);

    // Tapered tip using hull between the end of straight section and the tip
    hull() {
        // Cross-section at start of taper
        translate([straight_length - 0.01, 0, 0])
            cube([0.01, finger_width, finger_thickness]);

        // Cross-section at tip (narrower)
        translate([finger_length - 0.01, (finger_width - tip_width) / 2, 0])
            cube([0.01, tip_width, finger_thickness]);
    }
}

// Filleted finger body using minkowski with a small sphere
module finger_body_filleted() {
    // We shrink the body by fillet_r in all directions, then minkowski with sphere
    // to get rounded external edges
    minkowski() {
        // Shrunk body
        translate([fillet_r, fillet_r, fillet_r])
            difference() {
                // Straight section shrunk
                union() {
                    cube([straight_length - fillet_r, finger_width - 2 * fillet_r, finger_thickness - 2 * fillet_r]);

                    hull() {
                        translate([straight_length - fillet_r - 0.01, 0, 0])
                            cube([0.01, finger_width - 2 * fillet_r, finger_thickness - 2 * fillet_r]);

                        translate([finger_length - 2 * fillet_r - 0.01, (finger_width - tip_width) / 2, 0])
                            cube([0.01, tip_width - 2 * fillet_r, finger_thickness - 2 * fillet_r]);
                    }
                }

                // Nothing to subtract here
            }

        sphere(r = fillet_r);
    }
}

// FSR 402 pocket on the inner face (top face, Z = finger_thickness) at the tip
module fsr_pocket() {
    tip_center_x = finger_length - fsr_diameter / 2 - 3;
    tip_center_y = finger_width / 2;
    translate([tip_center_x, tip_center_y, finger_thickness - fsr_depth])
        cylinder(d = fsr_diameter, h = fsr_depth + 1);
}

// Copper electrode channels on the tip (inner face)
module electrode_channels() {
    tip_center_y = finger_width / 2;
    start_x = finger_length - electrode_length - 2;

    // Channel 1
    translate([start_x, tip_center_y - electrode_spacing / 2 - electrode_width, finger_thickness - electrode_depth])
        cube([electrode_length, electrode_width, electrode_depth + 1]);

    // Channel 2
    translate([start_x, tip_center_y + electrode_spacing / 2, finger_thickness - electrode_depth])
        cube([electrode_length, electrode_width, electrode_depth + 1]);
}

// Spring socket hole at the tip (through the contact face)
module spring_socket() {
    tip_center_x = finger_length - spring_socket_dia / 2 - 5;
    tip_center_y = finger_width / 2;
    translate([tip_center_x, tip_center_y, finger_thickness - spring_socket_depth])
        cylinder(d = spring_socket_dia, h = spring_socket_depth + 1);
}

// TCRT5000 sensor slots along the inner face, angled inward
module tcrt_slots() {
    for (i = [0 : tcrt_count - 1]) {
        slot_x = finger_length - tcrt_start - i * tcrt_spacing;
        slot_y = 0; // inner face is at y = 0 side

        translate([slot_x - tcrt_width / 2, 0, finger_thickness / 2])
            rotate([-(90 - tcrt_angle), 0, 0])
                translate([0, -tcrt_height / 2, -tcrt_depth])
                    cube([tcrt_width, tcrt_height, tcrt_depth + 2]);
    }
}

// Piezo disc recess on inner face near the base
module piezo_mount() {
    if (has_piezo) {
        px = piezo_offset;
        py = finger_width / 2;
        translate([px, py, finger_thickness - piezo_depth])
            cylinder(d = piezo_diameter, h = piezo_depth + 1);
    }
}

// Cable channel along the back (bottom face, Z = 0 side) from tip to base
module cable_channel() {
    translate([5, (finger_width - cable_width) / 2, -0.5])
        cube([finger_length - 10, cable_width, cable_depth + 0.5]);
}

// Servo horn mount holes at the base
module horn_mount() {
    base_center_x = 8;  // near the base end
    base_center_y = finger_width / 2;

    // Center hole for servo shaft boss
    translate([base_center_x, base_center_y, -1])
        cylinder(d = horn_center_hole, h = finger_thickness + 2);

    // Two screw holes at 7mm radius, positioned along X axis
    for (angle = [90, -90]) {
        hx = base_center_x + horn_hole_radius * cos(angle);
        hy = base_center_y + horn_hole_radius * sin(angle);
        translate([hx, hy, -1])
            cylinder(d = horn_hole_diameter, h = finger_thickness + 2);
    }
}

// Finger ID text embossed on the outer face (bottom, Z = 0)
module finger_label() {
    label_x = 20;
    label_y = finger_width / 2;
    // Text on the outer face (Z = 0), embossed (raised slightly)
    translate([label_x, label_y, 0])
        rotate([0, 0, 0])
            linear_extrude(height = text_depth)
                text(finger_id, size = text_size, halign = "center", valign = "center",
                     font = "Liberation Sans:style=Bold");
}

// ========================= MAIN ASSEMBLY ====================================

module gripper_finger() {
    difference() {
        union() {
            // Main finger body with fillets
            finger_body_filleted();

            // Raised finger ID text on outer face
            finger_label();
        }

        // FSR sensor pocket
        fsr_pocket();

        // Copper electrode channels
        electrode_channels();

        // Spring-loaded contact socket
        spring_socket();

        // TCRT5000 sensor slots
        tcrt_slots();

        // Piezo disc mount (conditional)
        piezo_mount();

        // Cable routing channel on back
        cable_channel();

        // Servo horn mounting holes
        horn_mount();
    }
}

// Render the finger
gripper_finger();
