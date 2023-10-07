
breite = 10;

difference() {
    cylinder(r=17, h= breite);
    translate([0,0,-1]) cylinder(r=14, h=(breite + 2));
    translate ([-17,0,-1]) cube([34,17,breite + 2]);
}

translate ([-17,0,0]) cube([3,34,breite]);
translate ([14,0,0]) cube([3,14,breite]);

difference() {
        translate ([15,11,0]) cube([17,3,breite]);
        #translate ([-14, 14,-1]) cube([47,24,breite + 2]);
}


/*
module aufhaengung() {
    difference() {
        translate ([0,0,0]) cube([20,100,10]);
        translate([10,10,-1]) cylinder(r=5, h=12);
    }
}

module schraube() {
    rotate(a=[270,0,0]) cylinder(r=2, h=22);
}


translate ([0,0,0]) aufhaengung();
translate ([0,0,30]) aufhaengung();

difference() {
    translate([0,100,0]) cube([120,20,40]);
    translate([50,99,10]) schraube();
    translate([50,99,30]) schraube();
    translate([80,99,10]) schraube();
    translate([80,99,30]) schraube();
}

module prism(l, w, h){
   rotate(a=[0,90,0]) polyhedron(
       points=[[0,0,0], [l,0,0], [l,w,0], [0,w,0], [0,w,h], [l,w,h]],
       faces=[[0,1,2,3],[5,4,3,2],[0,4,5,1],[0,3,4],[5,2,1]]
   );
}
translate([0,80,10]) cube([20,20,20]);
translate([20,80,00]) prism (-40,20,20);

difference() {
        translate([100,90,0]) cube([20,10,40]);
        translate([110,95,-1]) cylinder(r=2, h=42);
        translate([99,89,15]) cube([22,11,10]);
    
}
*/