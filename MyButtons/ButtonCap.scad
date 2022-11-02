$fn = 32;

Label = "0";
TextSize = 8;

module Clip(StemOuter, StemInter, ClipHeight) {
    union() {
        difference() {
            linear_extrude(ClipHeight)
                square(StemOuter, center=true);
            linear_extrude(ClipHeight)
                square(StemInter, center=true);
        }

        ClipDepth = 0.3;
        dimple_offset = StemInter / 2;
        dimple_height = ClipHeight - ClipDepth;
        translate([dimple_offset, 0, dimple_height])
            sphere(ClipDepth);
        translate([0, dimple_offset, dimple_height])
            sphere(ClipDepth);
        translate([-dimple_offset, 0, dimple_height])
            sphere(ClipDepth);
        translate([0, -dimple_offset, dimple_height])
            sphere(ClipDepth);
    }
}

module ButtonBase(ButtonWidth, ButtonThickness, StemLength) {
    union() {
        translate([0, 0, -ButtonThickness])
            linear_extrude(ButtonThickness)
                square(ButtonWidth, center=true);

        StemOuter = 5;
        StemInter = 4;
        ClipHeight = 3;
        linear_extrude(StemLength - ClipHeight)
            square(StemOuter, center=true);
        translate([0, 0, StemLength - ClipHeight])
            Clip(StemOuter, StemInter, ClipHeight);
    }
}

module Button(Label, TextSize) {
    ButtonWidth = 12;
    ButtonThickness = 3;
    StemLength = 6.6;

    difference() {
        ButtonBase(ButtonWidth, ButtonThickness, StemLength);

        translate([0, 0, -ButtonThickness])
            linear_extrude(ButtonThickness/4)
                mirror([0, 1, 0])
                    text(Label, size=TextSize, halign="center", valign="center");
    }
}

Button(Label, TextSize);