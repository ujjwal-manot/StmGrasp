// ============================================================================
// ir_led_ring.scad - IR LED Ring Mount for Photometric Stereo
// Part of HYDRA GRASP system for STEVAL-ROBKIT1
// All dimensions in mm
//
// PRINT ORIENTATION: Print flat with the ring face down (tabs pointing up).
// The angled LED holes print cleanly at 30 degrees without supports.
//
// Print Settings: 0.2mm layer height, 3 perimeters, 30% infill, PLA
// ============================================================================

$fn = 60;

// ========================= PARAMETERS =======================================

// Ring geometry
ring_od      = 40;   // outer diameter
ring_id      = 25;   // inner diameter (camera lens clearance)
ring_height  = 8;    // ring thickness/height

// Fillet on edges
fillet_r = 0.6;

// LED holes (5mm through-hole IR LEDs)
led_count    = 3;
led_hole_d   = 5.2;  // LED diameter + clearance
led_angle_down = 30;  // degrees angled downward toward workspace
led_positions = [0, 120, 240];  // angular positions in degrees

// Wire channels on back face
wire_channel_w = 2;
wire_channel_d = 1.5;

// Mounting tabs
tab_count    = 2;
tab_width    = 10;
tab_length   = 8;    // how far tab extends from ring OD
tab_thick    = 4;    // tab thickness (same as ring_height? no, thinner)
tab_hole_d   = 3.4;  // M3 clearance
tab_positions = [60, 240];  // angular positions for tabs (avoid LED positions)

// Labels
label_size  = 3;
label_depth = 0.5;

// ========================= DERIVED VALUES ===================================

ring_wall = (ring_od - ring_id) / 2;

// ========================= MODULES ==========================================

// Main ring body
module ring_body() {
    difference() {
        cylinder(d = ring_od, h = ring_height);
        translate([0, 0, -1])
            cylinder(d = ring_id, h = ring_height + 2);
    }
}

// Single LED hole angled downward
// The LED is inserted from the outer edge, angled down
module led_hole(angle_pos) {
    // Position on the ring
    mid_r = (ring_od / 2 + ring_id / 2) / 2;  // midpoint of ring wall

    // Rotate to angular position, then create angled hole
    rotate([0, 0, angle_pos])
        translate([mid_r, 0, ring_height / 2])
            // Angle downward (tilt toward Z-negative)
            rotate([led_angle_down, 0, 0])
                // Drill through the ring wall
                cylinder(d = led_hole_d, h = ring_wall + 4, center = true);
}

// All LED holes
module led_holes() {
    for (i = [0 : led_count - 1]) {
        led_hole(led_positions[i]);
    }
}

// Wire channels on the back face (top face when printing, bottom in use)
module wire_channels() {
    for (i = [0 : led_count - 1]) {
        ang = led_positions[i];
        // Radial channel from LED position to outer edge
        rotate([0, 0, ang])
            translate([ring_id / 2 - 1, -wire_channel_w / 2, ring_height - wire_channel_d])
                cube([ring_wall + 2, wire_channel_w, wire_channel_d + 1]);

        // Circumferential channel connecting to nearest tab exit
        // (simplified: just run the channel along the back face to the edge)
    }

    // Circumferential channel around the back face connecting all LED positions
    difference() {
        translate([0, 0, ring_height - wire_channel_d])
            difference() {
                cylinder(d = ring_od - 2, h = wire_channel_d + 1);
                translate([0, 0, -1])
                    cylinder(d = ring_od - 2 - 2 * wire_channel_w, h = wire_channel_d + 3);
            }
        // Only keep the channel, don't cut through inner wall
        translate([0, 0, ring_height - wire_channel_d - 1])
            cylinder(d = ring_id + 2, h = wire_channel_d + 3);
    }
}

// Mounting tabs with M3 holes
module mounting_tabs() {
    for (i = [0 : tab_count - 1]) {
        ang = tab_positions[i];
        rotate([0, 0, ang])
            translate([ring_od / 2 - 1, -tab_width / 2, 0]) {
                difference() {
                    // Tab body
                    hull() {
                        cube([1, tab_width, ring_height]);
                        translate([tab_length, tab_width * 0.15, 0])
                            cube([1, tab_width * 0.7, ring_height]);
                    }

                    // M3 hole
                    translate([tab_length / 2 + 1, tab_width / 2, -1])
                        cylinder(d = tab_hole_d, h = ring_height + 2);
                }
            }
    }
}

// LED labels embossed on the top face
module led_labels() {
    labels = ["IR1", "IR2", "IR3"];
    for (i = [0 : led_count - 1]) {
        ang = led_positions[i];
        label_r = ring_od / 2 - ring_wall / 2;

        rotate([0, 0, ang + 12])
            translate([label_r, 0, ring_height - label_depth])
                linear_extrude(height = label_depth + 0.5)
                    rotate([0, 0, -ang - 12])
                        text(labels[i], size = label_size, halign = "center",
                             valign = "center",
                             font = "Liberation Sans:style=Bold");
    }
}

// ========================= MAIN ASSEMBLY ====================================

module ir_led_ring() {
    difference() {
        union() {
            // Main ring
            ring_body();

            // Mounting tabs
            mounting_tabs();
        }

        // LED holes (angled)
        led_holes();

        // Wire channels
        wire_channels();

        // LED labels (embossed = subtracted from surface)
        led_labels();
    }
}

// Render
ir_led_ring();
