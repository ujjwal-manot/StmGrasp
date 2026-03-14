// ============================================================================
// sensor_backpack.scad - ESP32 + NodeMCU Backpack Module
// Part of HYDRA GRASP system for STEVAL-ROBKIT1
// All dimensions in mm
//
// PRINT ORIENTATION: Print flat on the bottom face (tray base down).
// Standoffs and clips print upward, no supports needed.
//
// Print Settings: 0.2mm layer height, 3 perimeters, 20% infill, PLA/PETG
// ============================================================================

$fn = 60;

// ========================= PARAMETERS =======================================

// Base tray
tray_width   = 80;
tray_depth   = 55;
tray_thick   = 3;    // base plate thickness
wall_height  = 5;    // short rim walls
wall_thick   = 1.5;

// Fillet
fillet_r = 1.0;

// ESP32 DevKit V1 mounting
// Board size approx 51 x 28mm, mounting holes at 48 x 23mm pattern
esp32_mount_x = 48;   // hole spacing along length
esp32_mount_y = 23;   // hole spacing along width
esp32_standoff_d = 5;  // standoff outer diameter
esp32_standoff_h = 3;  // standoff height above tray
esp32_hole_d    = 2.5; // hole diameter for M2.5 or self-tap
esp32_board_w   = 28;  // board width for clip reference
esp32_board_l   = 51;  // board length

// Clip dimensions (small lips to hold board edges)
clip_width  = 5;
clip_height = 4;   // total height above standoff
clip_lip    = 1.2; // how far the lip extends over the board
clip_thick  = 1.5;

// ESP32 position on tray (offset from corner)
esp32_offset_x = 5;
esp32_offset_y = 3;

// NodeMCU mounting
// Board size approx 48 x 26mm, mounting holes at 43 x 25mm pattern
nodemcu_mount_x = 43;
nodemcu_mount_y = 25;
nodemcu_standoff_d = 5;
nodemcu_standoff_h = 3;
nodemcu_hole_d    = 2.5;
nodemcu_board_w   = 26;
nodemcu_board_l   = 48;

// NodeMCU position (next to ESP32)
nodemcu_offset_x = 5;
nodemcu_offset_y = esp32_offset_y + esp32_board_w + 4;

// Breadboard slot
bb_width  = 55;   // length of mini breadboard
bb_depth  = 35;   // width of mini breadboard
bb_recess = 1;    // how deep the recess is

// Breadboard position (right side of tray)
bb_offset_x = tray_width - bb_width - 3;
bb_offset_y = (tray_depth - bb_depth) / 2;

// Wire exit slots (openings in walls)
wire_slot_w = 10;
wire_slot_h = wall_height;

// Mounting holes (M3) at corners
mount_hole_d    = 3.4;
mount_head_d    = 6.5;
mount_inset     = 5;  // distance from edge to hole center

// Label
label_text  = "HYDRA GRASP";
label_text2 = "ESP32 + NodeMCU";
label_size  = 3.5;
label_depth = 0.5;

// ========================= MODULES ==========================================

// Rounded box (same utility as gripper_base)
module rounded_box(size, r) {
    hull() {
        for (x = [r, size[0] - r]) {
            for (y = [r, size[1] - r]) {
                translate([x, y, 0])
                    cylinder(r = r, h = size[2]);
            }
        }
    }
}

// Base tray with rim walls
module tray_body() {
    difference() {
        union() {
            // Base plate
            rounded_box([tray_width, tray_depth, tray_thick], fillet_r);

            // Rim walls on all 4 sides
            // Front wall
            translate([0, 0, 0])
                rounded_box([tray_width, wall_thick, tray_thick + wall_height], fillet_r);
            // Back wall
            translate([0, tray_depth - wall_thick, 0])
                rounded_box([tray_width, wall_thick, tray_thick + wall_height], fillet_r);
            // Left wall
            translate([0, 0, 0])
                rounded_box([wall_thick, tray_depth, tray_thick + wall_height], fillet_r);
            // Right wall
            translate([tray_width - wall_thick, 0, 0])
                rounded_box([wall_thick, tray_depth, tray_thick + wall_height], fillet_r);
        }

        // Wire exit slots on all 4 sides (centered on each wall)
        // Front
        translate([(tray_width - wire_slot_w) / 2, -1, tray_thick])
            cube([wire_slot_w, wall_thick + 2, wire_slot_h + 1]);
        // Back
        translate([(tray_width - wire_slot_w) / 2, tray_depth - wall_thick - 1, tray_thick])
            cube([wire_slot_w, wall_thick + 2, wire_slot_h + 1]);
        // Left
        translate([-1, (tray_depth - wire_slot_w) / 2, tray_thick])
            cube([wall_thick + 2, wire_slot_w, wire_slot_h + 1]);
        // Right
        translate([tray_width - wall_thick - 1, (tray_depth - wire_slot_w) / 2, tray_thick])
            cube([wall_thick + 2, wire_slot_w, wire_slot_h + 1]);

        // Additional wire slots offset to the sides
        // Front left
        translate([12, -1, tray_thick])
            cube([wire_slot_w, wall_thick + 2, wire_slot_h + 1]);
        // Front right
        translate([tray_width - 12 - wire_slot_w, -1, tray_thick])
            cube([wire_slot_w, wall_thick + 2, wire_slot_h + 1]);
    }
}

// Board standoff post
module standoff(d, h, hole_d) {
    difference() {
        cylinder(d = d, h = h);
        translate([0, 0, -1])
            cylinder(d = hole_d, h = h + 2);
    }
}

// Board edge clip
module board_clip(board_thickness = 1.6) {
    // Vertical post
    cube([clip_thick, clip_width, clip_height]);
    // Lip overhang
    translate([-clip_lip, 0, clip_height - board_thickness - 0.3])
        cube([clip_lip + clip_thick, clip_width, 1.2]);
}

// ESP32 DevKit V1 mount
module esp32_mount() {
    translate([esp32_offset_x, esp32_offset_y, tray_thick]) {
        // 4 standoff posts at mounting hole pattern
        positions = [
            [0, 0],
            [esp32_mount_x, 0],
            [0, esp32_mount_y],
            [esp32_mount_x, esp32_mount_y]
        ];

        for (pos = positions) {
            translate([pos[0], pos[1], 0])
                standoff(esp32_standoff_d, esp32_standoff_h, esp32_hole_d);
        }

        // Edge clips (2 on each long side)
        // Left side clips
        translate([-clip_thick, 2, 0])
            board_clip();
        translate([-clip_thick, esp32_board_w - clip_width - 2, 0])
            board_clip();

        // Right side clips (mirrored)
        translate([esp32_board_l + 0.5, 2, 0])
            mirror([1, 0, 0])
                board_clip();
        translate([esp32_board_l + 0.5, esp32_board_w - clip_width - 2, 0])
            mirror([1, 0, 0])
                board_clip();
    }
}

// NodeMCU mount
module nodemcu_mount() {
    translate([nodemcu_offset_x, nodemcu_offset_y, tray_thick]) {
        // 4 standoff posts
        positions = [
            [0, 0],
            [nodemcu_mount_x, 0],
            [0, nodemcu_mount_y],
            [nodemcu_mount_x, nodemcu_mount_y]
        ];

        for (pos = positions) {
            translate([pos[0], pos[1], 0])
                standoff(nodemcu_standoff_d, nodemcu_standoff_h, nodemcu_hole_d);
        }

        // Edge clips
        translate([-clip_thick, 2, 0])
            board_clip();
        translate([-clip_thick, nodemcu_board_w - clip_width - 2, 0])
            board_clip();
        translate([nodemcu_board_l + 0.5, 2, 0])
            mirror([1, 0, 0])
                board_clip();
        translate([nodemcu_board_l + 0.5, nodemcu_board_w - clip_width - 2, 0])
            mirror([1, 0, 0])
                board_clip();
    }
}

// Breadboard recessed slot
module breadboard_slot() {
    translate([bb_offset_x, bb_offset_y, tray_thick - bb_recess])
        cube([bb_width, bb_depth, bb_recess + 0.1]);
}

// Corner mounting holes
module mounting_holes() {
    positions = [
        [mount_inset, mount_inset],
        [tray_width - mount_inset, mount_inset],
        [mount_inset, tray_depth - mount_inset],
        [tray_width - mount_inset, tray_depth - mount_inset]
    ];

    for (pos = positions) {
        translate([pos[0], pos[1], -1])
            cylinder(d = mount_hole_d, h = tray_thick + 2);
    }
}

// Label text on the front wall
module label() {
    // Text on the front wall exterior
    translate([tray_width / 2, -0.1, tray_thick + wall_height / 2 + 1])
        rotate([90, 0, 0])
            linear_extrude(height = label_depth + 0.1)
                text(label_text, size = label_size, halign = "center",
                     valign = "center",
                     font = "Liberation Sans:style=Bold");

    translate([tray_width / 2, -0.1, tray_thick + wall_height / 2 - 3])
        rotate([90, 0, 0])
            linear_extrude(height = label_depth + 0.1)
                text(label_text2, size = label_size * 0.7, halign = "center",
                     valign = "center",
                     font = "Liberation Sans");
}

// ========================= MAIN ASSEMBLY ====================================

module sensor_backpack() {
    difference() {
        union() {
            // Base tray with walls
            tray_body();

            // ESP32 standoffs and clips
            esp32_mount();

            // NodeMCU standoffs and clips
            nodemcu_mount();

            // Label (raised text on front wall)
            label();
        }

        // Breadboard recess
        breadboard_slot();

        // Corner mounting holes
        mounting_holes();
    }
}

// Render
sensor_backpack();
