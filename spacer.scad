height = 1;
outer_radius = 2;
inner_radius = 1;

difference() {
    cylinder(h=height, r=outer_radius, $fn=360);
    cylinder(h=height, r=inner_radius, $fn=360);
}