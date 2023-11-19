module loch() {
    cube([20,20,11]);
}


module aufhaengung() {
    difference() {
        translate ([0,0,0]) cube([51,71,9]);
        translate ([5,5,-1]) loch();
        translate ([26,5,-1]) loch();
        translate ([5,46,-1]) loch();
        translate ([26,46,-1]) loch();   
        translate ([-1,28,4]) cube([53,15,6]);             
    }
}

aufhaengung();