$fn = 32;

linear_extrude(5.5)
    difference() {
        union() {
            translate([25.5, 6.6])
                circle(5);
            translate([111.8, 6.6])
                circle(5);
            translate([25.6, 59.4])
                circle(5);
            translate([111.9, 59.4])
                circle(5);
        }
        
        union() {
            translate([25.5, 6.6])
                circle(2.5);
            translate([111.8, 6.6])
                circle(2.5);
            translate([25.6, 59.4])
                circle(2.5);
            translate([111.9, 59.4])
                circle(2.5);
        }
    }

translate([0, 0, -0.8])
    linear_extrude(0.8)
        difference() {

            square([117.2, 73]);
            
            union() {

                translate([25.5, 6.6])
                    circle(2.5);
                translate([111.8, 6.6])
                    circle(2.5);
                translate([25.6, 59.4])
                    circle(2.5);
                translate([111.9, 59.4])
                    circle(2.5);

                translate([25.5, 36.5])
                    square([18, 15], center=true);
                
                rows = [63.1, 45.3, 27.5, 9.8];
                cols = [57.4, 71.6, 85.9, 100.1];
                
                for (row = rows) {
                    for (col = cols) {
                        translate([col, row])
                            square(6, center=true);
                    }
                }

                col1 = 8.3; col2 = 43.2;
                translate([col1, rows[0]])
                    square(6, center=true);
                translate([col1, rows[3]])
                    square(6, center=true);
                translate([col2, rows[0]])
                    square(6, center=true);
                translate([col2, rows[3]])
                    square(6, center=true);
                
                translate([col1, 49.2])
                    circle(3.5);
                translate([col1, 23.5])
                    circle(3.5);
            }
        }