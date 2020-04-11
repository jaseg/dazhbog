eps=0.01;

nut_dia = 5.4;
nut_depth=5.5;

bx=71.12;
by=35.56;

w=10;
h=5;
s=30;
d=w;
bh=8;
sd=4;

module qc(a, b, d, h) {
    translate([-a, -b, 0]) cylinder(d=d, h=h);
    translate([-a,  b, 0]) cylinder(d=d, h=h);
    translate([ a, -b, 0]) cylinder(d=d, h=h);
    translate([ a,  b, 0]) cylinder(d=d, h=h);
}

module foo() {
    difference() {
        union() {
            translate([0, by/2, h/2]) cube([2*s+w, w, h], center=true);
            translate([0, -by/2, h/2]) cube([2*s+w, w, h], center=true);
            translate([-bx/2, 0, h/2]) cube([w, by, h], center=true);
            translate([bx/2, 0, h/2]) cube([w, by, h], center=true);

            qc(a=bx/2, b=by/2, d=d, h=bh);
            qc(a=s, b=by/2, d=d, h=bh);
        }
        translate([0, 0, -eps]) {
            qc(a=bx/2, b=by/2, d=sd, h=bh+2*eps);
            qc(a=s, b=by/2, d=sd, h=bh+2*eps);
            translate([0, 0, bh-nut_depth+eps]) qc(a=bx/2, b=by/2, d=nut_dia, h=nut_depth+eps);
            qc(a=s, b=by/2, d=nut_dia, h=nut_depth+eps);
        }
    }
}

foo($fn=25);