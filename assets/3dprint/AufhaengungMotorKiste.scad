

difference() {
    union() {
        cube([26.5,26.5,9]);
    }
    translate([22,5,-1]) cylinder(r=1.6, h=11, $fn=15);
    translate([8,5,-1]) cylinder(r=1.6, h=11, $fn=15);
    translate([8.5,21,-1]) cylinder(r=1.6, h=11, $fn=15);
    translate([22,21,-1]) cylinder(r=1.6, h=11, $fn=15);
    translate([15,13,-1]) cylinder(r=5, h=11, $fn=15);
}
/*
translate([-11,-11,0] ) {
    #cube([22,22,20]);
}

translate([-6,-6,0] ) {
    #cube([12,12,10]);
}*/