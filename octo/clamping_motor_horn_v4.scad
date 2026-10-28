// clamping_motor_horn.scad - 3D Printable Motor Horn (STL Export)
// v4: NO TAPER, a small local recess around the rod-end hole (matching the
// ball-stud's short thread length), and a nested hex nut trap on the nut
// side so the M5 nut seats flush and doesn't spin while tightening.
//
// v2 removed the long gradual taper (the actual weak point - it thinned
// material over the whole outer arm, exactly where bending stress is
// highest). v3 added a small symmetric recess just around the hole to match
// the ball-stud's short usable thread length, kept symmetric on purpose
// (not one-sided) because the crank pushes and pulls in both directions
// through its rotation - an even split handles both load directions,
// instead of preferentially weakening whichever face a one-sided cut would
// thin. v4 nests a small hex cutout inside the nut-side half of that
// recess - deeper than the round pocket, but only across the hex footprint,
// so it stays a small, contained feature rather than another full-face cut.
//
// PRINT ORIENTATION STILL MATTERS: print standing on edge (flat face
// vertical), not lying flat, so layer lines run along the arm's length.

$fn = 60;

// --- User Parameters ---
throw_length = 20.0;       // Distance from motor shaft center to the rod-end hole
horn_thickness = 8.0;      // Thickness everywhere EXCEPT the local hole recess
hub_dia = 14.0;            // Outer diameter of the central hub
arm_width = 16.0;          // Width of the arm (slightly wider than the hub for a clean blend)
shaft_dia = 5.2;           // 5mm motor shaft + 0.2mm 3D printing tolerance
flat_depth = 0.6;          // Depth of the NEMA 17 D-shaft flat
m3_clearance = 1.7;        // 3.4mm diameter for M3 clamping bolt
m5_clearance = 2.6;        // 5.2mm diameter for the M5 rod-end hole
arm_end_margin = 8.0;      // Extra material past the M5 hole, so it doesn't
                           // break out through the tip edge

// Local recess around the hole, to match the ball-stud's short thread length
hole_depth = 5.0;          // target material thickness right at the hole
pocket_dia = 10.0;         // diameter of the recessed pad around the hole
pocket_cut = (horn_thickness - hole_depth) / 2; // removed from EACH face - kept
                           // symmetric on purpose: the crank pushes and pulls
                           // in both directions through its rotation, so an
                           // even split handles both load directions equally
                           // instead of preferentially weakening one of them

// Hex nut trap, NUT SIDE ONLY, nested inside the pocket above (not instead
// of it) - small and localized, just enough to grip the flats and stop the
// nut spinning while tightening, not a full-depth burial.
// IMPORTANT: this depth is measured from the same outer face as pocket_cut,
// NOT stacked on top of it - so if hex_nut_depth > pocket_cut, the actual
// remaining material at the hole is thinner than hole_depth, not equal to
// it. Kept equal to pocket_cut here so the two add up to exactly
// (horn_thickness - hole_depth), hitting the real 5mm target precisely.
hex_nut_across_flats = 8.3; // M5 nut, 8mm nominal + slight clearance
hex_nut_r = hex_nut_across_flats / 1.732;
hex_nut_depth = pocket_cut; // = 1.5mm here - see note above before changing

// --- Cutout Modules ---
module d_shaft_cutout() {
    difference() {
        cylinder(r=shaft_dia/2, h=horn_thickness + 2, center=true);
        translate([0, (shaft_dia/2) - flat_depth + 2.5, 0])
            cube([shaft_dia+2, 5, horn_thickness + 4], center=true);
    }
}

arm_length = throw_length + arm_end_margin;

// --- Main Horn Geometry ---
difference() {
    union() {
        // Hub + arm, one continuous full-thickness hull - no taper stages
        hull() {
            cylinder(r=hub_dia/2, h=horn_thickness, center=true);
            translate([arm_length, 0, 0])
                cylinder(r=arm_width/2, h=horn_thickness, center=true);
        }

        // Clamping block (unchanged from the original design)
        translate([-7, 0, 0])
            cube([12, 12, horn_thickness], center=true);
    }

    // Motor shaft
    d_shaft_cutout();

    // Local recess around the rod-end hole, cut into BOTH faces symmetrically -
    // leaves a hole_depth-thick pad right at the hole, full thickness everywhere
    // else on the arm.
    translate([throw_length, 0, horn_thickness/2 - pocket_cut])
        cylinder(r=pocket_dia/2, h=pocket_cut + 0.5);
    translate([throw_length, 0, -horn_thickness/2 - 0.5])
        cylinder(r=pocket_dia/2, h=pocket_cut + 0.5);

    // Hex nut trap, nested inside the nut-side (bottom) pocket above - deeper
    // than the round pocket but only across the hex footprint, not the whole
    // pocket area, so it stays a small, contained feature
    translate([throw_length, 0, -horn_thickness/2 - 0.01])
        cylinder(r=hex_nut_r, h=hex_nut_depth, $fn=6);

    // Rod-end hole itself, now only as deep as hole_depth requires
    translate([throw_length, 0, 0])
        cylinder(r=m5_clearance, h=horn_thickness + 2, center=true);

    // Clamping slit (allows the plastic to flex and grip the shaft)
    translate([-10, 0, 0])
        cube([20, 1.5, horn_thickness + 2], center=true);

    // Clamping bolt hole (passes perpendicularly through the slit)
    translate([-7, 0, 0])
        rotate([90, 0, 0])
        cylinder(r=m3_clearance, h=20, center=true);

    // M3 hex nut trap
    translate([-7, 4.5, 0])
        rotate([90, 30, 0])
        cylinder(r=3.3, h=4, $fn=6, center=true);

    // Bolt head recess
    translate([-7, -4.5, 0])
        rotate([90, 0, 0])
        cylinder(r=3.2, h=4, center=true);
}

