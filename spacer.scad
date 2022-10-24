$fn=32;

module spacer(height, inner_radius = 5, thickness = 2) {
    outer_radius = thickness + inner_radius;
    difference() {
        cylinder(h=height, r=outer_radius);
        cylinder(h=height, r=inner_radius);
    }
}

spacer(5);