$fn = 32;

module hexagon(point_to_point=4) {
    rotate(30, [0, 0, 1])
        circle(point_to_point, $fn=6);
}

module Post(Width=5) {
    difference() {
        hexagon(Width);
        circle(2.5); // 5mm diamter
    }
}

module front_panel_plate() {
    union() {

        PlateThickness = 2;
        
        AltCols = [8.3, 43.2];
        LEDRows = [23.5, 49.2];
        MountingCols = [25.6, 111.8];
        MountingRows = [13.6, 66.4];

        difference() {
            PostHeight = 5;
            linear_extrude(PostHeight)
                union() {
                    // Mounting Posts
                    for (MountingCol = MountingCols) {
                        for (MountingRow = MountingRows) {
                            translate([MountingCol, MountingRow])
                                Post();
                        }
                    }
                    
                    // LED Posts
                    for (LEDRow = LEDRows) {
                        translate([AltCols[0], LEDRow])
                            difference() {
                                circle(5); // 10mm diameter
                                circle(3.5); // 7mm diameter
                            }
                    }
                }
                
                
            // Cut Out in the Mounting Posts for the Standoffs
            StandoffDepth = 4.2;
            linear_extrude(StandoffDepth)
                union() {
                    for (MountingCol = MountingCols) {
                        for (MountingRow = MountingRows) {
                            translate([MountingCol, MountingRow])
                                Post(4);
                        }
                    }
                }
        }
        
        translate([0, 0, -PlateThickness])
            linear_extrude(PlateThickness)
                difference() {

                    // Main Plate
                    TotalWidth = 73;
                    TotalLength = 117.3;
                    square([TotalLength, TotalWidth]);
                    
                    union() {

                        // Mounting Holes
                        for (MountingCol = MountingCols) {
                            for (MountingRow = MountingRows) {
                                translate([MountingCol, MountingRow])
                                    hexagon();
                            }
                        }

                        // Rotary Encoder Cut-Out
                        translate([25.5, 36.5])
                            circle(4); // 8mm  Diameter
                        
                        // Main Keypad Matrix
                        ButtonHole = 6.4;
                        rows = [63.1, 45.3, 27.5, 9.8];
                        cols = [57.4, 71.6, 85.9, 100.1];
                        for (row = rows) {
                            for (col = cols) {
                                translate([col, row])
                                    square(ButtonHole, center=true);
                            }
                        }

                        // Secondary Buttons
                        
                        for (AltCol = AltCols) {
                            translate([AltCol, rows[0]])
                                square(ButtonHole, center=true);
                            translate([AltCol, rows[3]])
                                square(ButtonHole, center=true);
                        }

                        
                        // LED Cut-Outs
                        LEDHole = 3.5;
                        for (LEDRow = LEDRows) {
                            translate([AltCols[0], LEDRow])
                                circle(LEDHole);
                        }
                    }
                }
    }
}

front_panel_plate();