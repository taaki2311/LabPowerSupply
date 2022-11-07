$fn = 32;

Label = "8";
TextSize = 10;

module Clip(StemOuter, StemInter, ClipHeight) {
    union() {
        difference() {
            linear_extrude(ClipHeight)
                square(StemOuter, center=true);
            linear_extrude(ClipHeight)
                square(StemInter, center=true);
        }
        
        intersection() {
            union() {
                ClipDepth = 0.3;
                IndentHeight = 1;
                BallRadius = (IndentHeight ^ 2) + (ClipDepth / 2);
                BallOffset = (StemInter / 2) + (BallRadius - ClipDepth);
                BallHeight = ClipHeight - BallRadius;
                translate([BallOffset, 0, BallHeight])
                    sphere(BallRadius);
                translate([0, BallOffset, BallHeight])
                    sphere(BallRadius);
                translate([-BallOffset, 0, BallHeight])
                    sphere(BallRadius);
                translate([0, -BallOffset, BallHeight])
                    sphere(BallRadius);
            }
            
            linear_extrude(ClipHeight)
                square(StemInter, center=true);
        }
    }
}

module ButtonBase(ButtonWidth, ButtonThickness, StemLength) {
    union() {
        translate([0, 0, -ButtonThickness])
            linear_extrude(ButtonThickness)
                square(ButtonWidth, center=true);

        StemOuter = 5.2;
        StemInter = 4.2;
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
    StemLength = 7;

    difference() {
        ButtonBase(ButtonWidth, ButtonThickness, StemLength);

        translate([0, 0, -ButtonThickness])
            linear_extrude(ButtonThickness/4)
                mirror([0, 1, 0])
                    text(Label, size=TextSize, halign="center", valign="center", font="Liberation Sans:style=Bold");
    }
}
Button(Label, TextSize);
