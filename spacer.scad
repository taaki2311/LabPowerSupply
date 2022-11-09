$fn=32;

module spacer(height = 10, inner_radius = 2, thickness = 1) {
    outer_radius = thickness + inner_radius;
    difference() {
        cylinder(h=height, r=outer_radius);
        cylinder(h=height, r=inner_radius);
    }
}

spacer();