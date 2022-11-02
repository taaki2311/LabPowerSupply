$fn = 32;

Label = "A";
TextSize = 5;

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
                BallRadius = (IndentHeight ^ 2) / 8 + (ClipDepth / 2);
                BallLargeOffset = (StemInter / 2) + (BallRadius - ClipDepth);
                BallSmallOffset = (StemInter / 4);
                BallHeight = ClipHeight - ClipDepth;
                translate([BallLargeOffset, BallSmallOffset, BallHeight])
                    sphere(BallRadius);
                translate([BallSmallOffset, BallLargeOffset, BallHeight])
                    sphere(BallRadius);
                translate([-BallLargeOffset, BallSmallOffset, BallHeight])
                    sphere(BallRadius);
                translate([BallSmallOffset, -BallLargeOffset, BallHeight])
                    sphere(BallRadius);
                translate([BallLargeOffset, -BallSmallOffset, BallHeight])
                    sphere(BallRadius);
                translate([-BallSmallOffset, BallLargeOffset, BallHeight])
                    sphere(BallRadius);
                translate([-BallLargeOffset, -BallSmallOffset, BallHeight])
                    sphere(BallRadius);
                translate([-BallSmallOffset, -BallLargeOffset, BallHeight])
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
    ButtonWidth = 11;
    ButtonThickness = 3;
    StemLength = 7;

    difference() {
        ButtonBase(ButtonWidth, ButtonThickness, StemLength);

        translate([0, 0, -ButtonThickness])
            linear_extrude(ButtonThickness/4)
                mirror([0, 1, 0])
                    text(Label, size=TextSize, halign="center", valign="center");
    }
}

Button(Label, TextSize);