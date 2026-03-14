// ============================================================================
// gripper_base.scad - Main Gripper Frame with Servo Mounts
// Part of HYDRA GRASP system for STEVAL-ROBKIT1
// All dimensions in mm
//
// PRINT ORIENTATION: Print flat on the base plate (bottom face down).
// The U-shape opens upward. No supports needed - all overhangs are < 45 deg.
//
// Print Settings: 0.2mm layer height, 3 perimeters, 25% infill, PLA/PETG
// ============================================================================

$fn = 60;

// ========================= PARAMETERS =======================================

// Overall frame dimensions (U-shape)
frame_width   = 80;   // total width (left to right)
frame_depth   = 50;   // front to back
frame_height  = 30;   // total height
wall_thick    = 4;    // wall thickness

// Base/mounting plate
base_width    = 80;
base_depth    = 60;
base_thick    = 3;

// Fillet radius
fillet_r = 1.0;

// SG90 Servo dimensions (with clearance)
servo_w = 23.5;   // width (along servo length)
servo_h = 12.5;   // height (narrow dimension)
servo_d = 29.5;   // depth
// SG90 mounting tab: extends 2.5mm each side, 1mm thick, at ~15.5mm from bottom
servo_tab_ext     = 2.5;   // how far tab extends beyond body
servo_tab_thick   = 1.2;   // tab thickness
servo_tab_height  = 15.5;  // height from servo bottom to center of tab
servo_tab_hole_d  = 2.2;   // M2 screw hole in tab
// Servo output shaft: offset 6mm from one edge, centered on width
servo_shaft_d     = 5;     // shaft hole diameter
servo_shaft_offset = 6;    // from edge of servo body

// Spacing between the two finger servos
servo_spacing = 6;   // gap between the two servo pockets

// Wrist servo (third SG90, oriented 90 degrees at the back)
wrist_servo = true;

// WS2812B LED strip channel
led_strip_width = 11;   // 10mm strip + clearance
led_strip_depth = 4;    // 3mm strip + clearance

// Mounting holes (M3) for attaching to robot chassis
mount_hole_d     = 3.4;   // M3 + clearance
mount_hole_head  = 6.5;   // M3 cap head clearance
mount_pattern_x  = 60;    // spacing along X
mount_pattern_y  = 40;    // spacing along Y
mount_counterbore = 2;    // counterbore depth for screw heads

// Cable management
cable_channel_w = 5;
cable_channel_h = 4;

// Ventilation holes
vent_hole_d    = 3;
vent_rows      = 3;
vent_cols      = 4;
vent_spacing_x = 8;
vent_spacing_y = 7;

// ========================= DERIVED VALUES ===================================

// Total width occupied by two servos side by side
servo_block_w = 2 * servo_w + servo_spacing;
// Center the servo block in the frame
servo_block_offset_x = (frame_width - servo_block_w) / 2;

// U-shape inner cavity
inner_width  = frame_width - 2 * wall_thick;
inner_depth  = frame_depth - wall_thick;   // open at front

// ========================= MODULES ==========================================

// Rounded box primitive
module rounded_box(size, r) {
    // A box with rounded vertical edges
    hull() {
        for (x = [r, size[0] - r]) {
            for (y = [r, size[1] - r]) {
                translate([x, y, 0])
                    cylinder(r = r, h = size[2]);
            }
        }
    }
}

// U-shaped frame (open at front, i.e., Y = frame_depth side)
module u_frame() {
    difference() {
        // Outer block
        rounded_box([frame_width, frame_depth, frame_height], fillet_r);

        // Inner cavity (the U opening)
        translate([wall_thick, -1, base_thick])
            cube([inner_width, inner_depth + 1, frame_height]);
    }
}

// Single SG90 servo pocket with tab slots and shaft hole
// Origin at bottom-left-front corner of the pocket
module servo_pocket(shaft_side = "top") {
    // Main body cavity
    cube([servo_w, servo_d, servo_h]);

    // Tab slots on each side (left and right of the narrow dimension)
    // Tabs are along the depth at servo_tab_height
    translate([-servo_tab_ext, 0, servo_tab_height - servo_tab_thick / 2])
        cube([servo_w + 2 * servo_tab_ext, servo_d, servo_tab_thick]);

    // Tab screw holes (through the frame wall)
    for (side = [-1, 1]) {
        tx = (side < 0) ? -servo_tab_ext - 5 : servo_w + servo_tab_ext;
        translate([tx, servo_d / 2, servo_tab_height])
            rotate([0, 90, 0])
                cylinder(d = servo_tab_hole_d, h = 10);
    }

    // Output shaft hole at the top
    if (shaft_side == "top") {
        translate([servo_shaft_offset, servo_d / 2, servo_h - 1])
            cylinder(d = servo_shaft_d, h = wall_thick + 5);
    }
}

// Two finger servo pockets side by side
module finger_servo_pockets() {
    // Left servo
    translate([servo_block_offset_x, (frame_depth - servo_d) / 2,
               base_thick])
        servo_pocket();

    // Right servo
    translate([servo_block_offset_x + servo_w + servo_spacing,
               (frame_depth - servo_d) / 2, base_thick])
        servo_pocket();

    // Shaft holes through the top for both servos
    // Left servo shaft
    translate([servo_block_offset_x + servo_shaft_offset,
               (frame_depth - servo_d) / 2 + servo_d / 2,
               frame_height - 5])
        cylinder(d = servo_shaft_d, h = 10);

    // Right servo shaft
    translate([servo_block_offset_x + servo_w + servo_spacing + servo_shaft_offset,
               (frame_depth - servo_d) / 2 + servo_d / 2,
               frame_height - 5])
        cylinder(d = servo_shaft_d, h = 10);
}

// Wrist servo pocket (rotated 90 degrees, at the back)
module wrist_servo_pocket() {
    if (wrist_servo) {
        // Position at the back wall, centered horizontally, rotated so servo
        // is vertical with shaft pointing up
        translate([frame_width / 2, frame_depth - wall_thick + 1, base_thick + 2])
            rotate([0, 0, 90])
                translate([-servo_w / 2, -servo_d / 2, 0])
                    servo_pocket();

        // Shaft hole for wrist servo through top
        translate([frame_width / 2, frame_depth - wall_thick - servo_d / 2 + 1,
                   frame_height - 5])
            cylinder(d = servo_shaft_d, h = 10);
    }
}

// WS2812B LED strip channel along the top front edge
module led_channel() {
    // Along the top of the front wall
    translate([(frame_width - inner_width + 4) / 2, -1,
               frame_height - led_strip_depth])
        cube([frame_width - wall_thick * 2, led_strip_width, led_strip_depth + 1]);
}

// Mounting plate with holes
module mounting_plate() {
    // Extended base plate
    difference() {
        translate([0, 0, 0])
            rounded_box([base_width, base_depth, base_thick], fillet_r);

        // 4 mounting holes at corners in rectangular pattern
        for (x = [(base_width - mount_pattern_x) / 2,
                  (base_width + mount_pattern_x) / 2]) {
            for (y = [(base_depth - mount_pattern_y) / 2,
                      (base_depth + mount_pattern_y) / 2]) {
                // Through hole
                translate([x, y, -1])
                    cylinder(d = mount_hole_d, h = base_thick + 2);

                // Counterbore for screw head (from bottom)
                translate([x, y, -1])
                    cylinder(d = mount_hole_head, h = mount_counterbore + 1);
            }
        }
    }
}

// Cable management channels connecting servo pockets to rear cable exit
module cable_channels() {
    // Channel from servo area to back
    translate([(frame_width - cable_channel_w) / 2, frame_depth - wall_thick - 1,
               base_thick])
        cube([cable_channel_w, wall_thick + 2, cable_channel_h]);

    // Left cable channel
    translate([servo_block_offset_x + servo_w / 2 - cable_channel_w / 2,
               frame_depth - wall_thick - 1, base_thick])
        cube([cable_channel_w, wall_thick + 2, cable_channel_h]);

    // Right cable channel
    translate([servo_block_offset_x + servo_w + servo_spacing + servo_w / 2 - cable_channel_w / 2,
               frame_depth - wall_thick - 1, base_thick])
        cube([cable_channel_w, wall_thick + 2, cable_channel_h]);
}

// Ventilation holes on the side walls
module ventilation_holes() {
    // Left side
    for (r = [0 : vent_rows - 1]) {
        for (c = [0 : vent_cols - 1]) {
            vx = -1;
            vy = 10 + c * vent_spacing_x;
            vz = base_thick + 5 + r * vent_spacing_y;
            if (vz < frame_height - 3 && vy < frame_depth - 5) {
                translate([vx, vy, vz])
                    rotate([0, 90, 0])
                        cylinder(d = vent_hole_d, h = wall_thick + 2);
            }
        }
    }

    // Right side
    for (r = [0 : vent_rows - 1]) {
        for (c = [0 : vent_cols - 1]) {
            vx = frame_width - wall_thick - 1;
            vy = 10 + c * vent_spacing_x;
            vz = base_thick + 5 + r * vent_spacing_y;
            if (vz < frame_height - 3 && vy < frame_depth - 5) {
                translate([vx, vy, vz])
                    rotate([0, 90, 0])
                        cylinder(d = vent_hole_d, h = wall_thick + 2);
            }
        }
    }
}

// ========================= MAIN ASSEMBLY ====================================

module gripper_base() {
    difference() {
        union() {
            // U-shaped frame
            u_frame();

            // Extended mounting plate (base extends backward)
            translate([0, 0, 0])
                mounting_plate();
        }

        // Subtract servo pockets
        finger_servo_pockets();

        // Subtract wrist servo pocket
        wrist_servo_pocket();

        // WS2812B LED channel
        led_channel();

        // Cable management channels
        cable_channels();

        // Ventilation holes
        ventilation_holes();
    }
}

// Render
gripper_base();
