w = 20;
h = w;
d = 7;
hole_dia = 6;
hole_d = 4;
hole_fringe = 1.5;
fringe_extra = 1.0;
eps = 0.001;

module stopper() {
    difference() {
        union() {
        translate([-w/2, -h/2, 0]) cube([w, h, d]);
        translate([0, 0, d-eps]) cylinder(d1=hole_dia+2*hole_fringe+2*fringe_extra, d2=hole_dia+2*fringe_extra, h=hole_fringe);
        }
        translate([0, 0, d+hole_fringe-hole_d]) cylinder(d=hole_dia, h=hole_d+eps);
    }
}

stopper($fn=25);