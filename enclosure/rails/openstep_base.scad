eps=0.01;

nut_dia = 5.3;
nut_depth=5.5;

module nut_base(w=10, o) {
    difference() {
        cylinder(d=w, h=nut_depth+o);
        translate([0, 0, o-eps]) cylinder(d=nut_dia, h=nut_depth+2*eps);
    }
}

module raspi_holder(w, h, x=4) {
    difference() {
        union() {
            cube([50+w, 50+w, h]);
            
            translate([w/2, w/2, 0]) union() {
                nut_base(o=h);
                translate([50, 0, 0]) {
                    nut_base(o=h);
                    translate([0, 50, 0]) nut_base(o=h);
                }
                translate([0, 50, 0]) nut_base(o=h);
            }
        }
        translate([w, w, -eps]) cube([50-w, 50-w, h+2*eps]);
        
        
        translate([w/2, w/2, -eps]) {
            cylinder(d=x, h=h+2*eps);
            translate([50, 0, 0]) {
                cylinder(d=x, h=h+2*eps);
                translate([0, 50, 0]) cylinder(d=x, h=h+2*eps);
            }
            translate([0, 50, 0]) cylinder(d=x, h=h+2*eps);
        }
    }
}

module rev_nut_base(w=10, h=5, a=45, x=4, th=7, tw=10) {
    rotate([0, 0, a])
    translate([-w/2, -w, 0]) difference() {
        union() {
            cube([w, w, h]);
            translate([w/2, w, 0]) cylinder(d=tw, h=th);
        }
        translate([w/2, w, -eps]) {
            cylinder(d=nut_dia, h=nut_depth+2*eps);
            cylinder(d=x, h=th+2*eps);
        }
    }
}

module raspi_adapter(w=10, h=5, s=30) {
    translate([-(50+w)/2, -(50+w)/2, 0]) raspi_holder(w=w, h=h);
    difference() {
        union() {
            translate([-s, -(50+w)/2-5, 0]) rev_nut_base(a=-30-180);
            translate([ s, -(50+w)/2-5, 0]) rev_nut_base(a=-60-90);
            translate([-s,  (50+w)/2+5, 0]) rev_nut_base(a=-60+90);
            translate([ s,  (50+w)/2+5, 0]) rev_nut_base(a=-30);
        }
        cube([60, 60, 100], center=true);
    }
}

raspi_adapter($fn=25);
