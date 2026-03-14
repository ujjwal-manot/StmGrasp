// ============================================================================
// spring_contact_pad.scad - Spring-Loaded Contact Tip for Gripper Finger
// Part of HYDRA GRASP system for STEVAL-ROBKIT1
// All dimensions in mm
//
// PRINT ORIENTATION: Print standing upright (contact face up).
// The cylindrical body prints well vertically. The thin FSR disc at the
// top may need careful first-layer adhesion.
//
// QUANTITY: Print 2 (one per finger)
//
// Print Settings: 0.16mm layer height, 3 perimeters, 40% infill, PLA/PETG
// Use fine layer height for the thin contact disc.
// ============================================================================

$fn = 60;

// ========================= PARAMETERS =======================================

// Overall dimensions
pad_diameter  = 5.5;   // body diameter (fits in 6mm socket with 0.5mm clearance)
total_length  = 12;    // total pad length

// Contact surface (wider disc at top)
contact_dia    = 13;    // matches FSR diameter, wider than body
contact_thick  = 2;     // thickness of the contact disc
contact_chamfer = 0.5;  // chamfer on the disc edge

// Cylindrical body
body_length = total_length - contact_thick;   // 10mm body

// FSR pocket on contact face
fsr_pocket_dia   = 13;    // same as contact_dia
fsr_pocket_depth = 0.5;

// Electrode grooves on contact face
electrode_width   = 7;
electrode_depth   = 0.5;
electrode_spacing = 3;   // gap between grooves

// Spring retention lip
lip_height   = 2;     // height from bottom where lip is located
lip_diameter = 6.2;   // slightly wider than socket ID to retain spring
lip_width    = 0.8;   // how tall the lip ring is

// Spring reference
spring_od     = 5;     // spring outer diameter
spring_length = 10;    // free length of spring

// ========================= MODULES ==========================================

// Main cylindrical body
module pad_body() {
    // Lower cylindrical shaft (slides in the socket)
    cylinder(d = pad_diameter, h = body_length);

    // Contact disc at the top (wider than body)
    translate([0, 0, body_length]) {
        // Chamfered disc using hull
        hull() {
            cylinder(d = contact_dia - 2 * contact_chamfer,
                     h = contact_thick);
            translate([0, 0, contact_chamfer])
                cylinder(d = contact_dia,
                         h = contact_thick - contact_chamfer);
        }
    }
}

// FSR pocket on the contact face (top)
module fsr_pocket() {
    translate([0, 0, total_length - fsr_pocket_depth])
        cylinder(d = fsr_pocket_dia, h = fsr_pocket_depth + 1);
}

// Electrode grooves on the contact face
module electrode_grooves() {
    translate([0, 0, total_length - electrode_depth]) {
        // Groove 1
        translate([-electrode_width / 2,
                   -contact_dia / 2, 0])
            cube([electrode_width, (contact_dia - electrode_spacing) / 2,
                  electrode_depth + 1]);

        // Groove 2
        translate([-electrode_width / 2,
                   electrode_spacing / 2, 0])
            cube([electrode_width, (contact_dia - electrode_spacing) / 2,
                  electrode_depth + 1]);
    }
}

// Spring retention lip (a small ring/ridge near the bottom)
module spring_lip() {
    translate([0, 0, lip_height - lip_width / 2])
        cylinder(d = lip_diameter, h = lip_width);
}

// Chamfer at the bottom for easy insertion
module bottom_chamfer() {
    chamfer = 0.5;
    translate([0, 0, -0.01])
        difference() {
            cylinder(d = pad_diameter + 1, h = chamfer);
            cylinder(d1 = pad_diameter - 2 * chamfer, d2 = pad_diameter,
                     h = chamfer);
        }
}

// ========================= MAIN ASSEMBLY ====================================

module spring_contact_pad() {
    difference() {
        union() {
            // Main body with contact disc
            pad_body();

            // Spring retention lip
            spring_lip();
        }

        // FSR pocket on contact face
        fsr_pocket();

        // Electrode grooves
        electrode_grooves();

        // Bottom chamfer for easy insertion into socket
        bottom_chamfer();
    }
}

// Render
spring_contact_pad();

// Optional: uncomment to see spring for reference (not part of print)
// %translate([0, 0, -spring_length])
//     difference() {
//         cylinder(d = spring_od, h = spring_length);
//         translate([0, 0, -1])
//             cylinder(d = spring_od - 1.0, h = spring_length + 2);
//     }
