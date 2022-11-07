$fn = 32;

module Post(Width) {
    difference() {
        square(Width, center=true);
        circle(Width / 4);
    }
}

module Posts(Height) {
    linear_extrude(Height)
        union() {
            translate([25.6, 13.6])
                Post(10);
            translate([111.9, 13.6])
                Post(10);
            translate([25.5, 66.4])
                Post(10);
            translate([111.8, 66.4])
                Post(10);
        }
}

union() {

    PlateThickness = 0.5;
    PostHeight = 4.5;
    Posts(PostHeight);
    
    translate([0, 0, -PlateThickness])
        linear_extrude(PlateThickness)
            difference() {

                // Main Plate
                TotalHeight = 73;
                TotalWidth = 117.2;
                square([TotalWidth, TotalHeight]);
                
                union() {

                    // Mounting Holes
                    translate([25.5, 66.4])
                        circle(2.5);
                    translate([111.8, 66.4])
                        circle(2.5);
                    translate([25.6, 13.6])
                        circle(2.5);
                    translate([111.9, 13.6])
                        circle(2.5);

                    // Rotary Encoder Cut-Out
                    translate([25.5, 36.5])
                        circle(4);
                    
                    // Main Keypad Matrix
                    ButtonHole = 5.8;
                    rows = [63.1, 45.3, 27.5, 9.8];
                    cols = [57.4, 71.6, 85.9, 100.1];
                    for (row = rows) {
                        for (col = cols) {
                            translate([col, row])
                                square(ButtonHole, center=true);
                        }
                    }

                    // Secondary Buttons
                    col1 = 8.3; col2 = 43.2;
                    translate([col1, rows[0]])
                        square(ButtonHole, center=true);
                    translate([col1, rows[3]])
                        square(ButtonHole, center=true);
                    translate([col2, rows[0]])
                        square(ButtonHole, center=true);
                    translate([col2, rows[3]])
                        square(ButtonHole, center=true);
                    
                    // LED Cut-Outs
                    LEDHole = 3.5;
                    translate([col1, 49.2])
                        circle(LEDHole);
                    translate([col1, 23.5])
                        circle(LEDHole);
                }
            }
}