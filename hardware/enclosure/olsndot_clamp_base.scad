eps = 0.01;

module clamp(
    l=90,
    w=14,
    h=10,
    d1=30,
    d2=35,
    o=10,
    hdia1=5,
    hdia2=10,
    hd=5,
    nw=7.8,
    nd=3,
    notch_sf=0.75,
    notch_d=3,
    notch_a=10,
    notch_o=0.5,
    edge_a=5,
    edge_d=12) {
    translate([-h-l/2, -w/2, 0]) union() {
        translate([l+h-notch_d, 0, 0]) intersection() {
            difference() {
                cube([d1, w, d1]);
                //scale([notch_sf, 1, 1]) translate([(d1-h)/2, 0, h+(d1-h)/2]) rotate([0, -135, 0]) translate([0, -eps/2, 0]) cube([d1, w+eps, d1]);
                translate([notch_d, 0, h+notch_o]) mirror([1, 0, 0]) rotate([0, notch_a, 0]) translate([0, -eps/2, 0]) cube([d1, w+eps, d1]);
            }
            translate([d1-edge_a, 0, d1/2]) rotate([0, -90-45, 0]) cube([2*d1, w+eps, 2*d1]);
            translate([-d1-edge_d, 0]) cube([2*d1, w+eps, 2*d1]);
        }
        difference() {
            union() {
                cube ([l+h, w, h]);
                cube ([h, w, d2]);
            }
            rotate([0, 45, 0]) translate([-l/2, -eps/2, -d2/8*7]) cube([l, w+eps, d2]);
            translate([0, 0, d2]) rotate([0, 90+45, 0]) translate([-l/2, -eps/2, -d2/8*7]) cube([l, w+eps, d2]);
            translate([-eps/2, w/2, d2-o]) rotate([0, 90, 0]) union() {
                cylinder(d=6, h=l);
                cylinder(d=12, h=hd);
                translate([-11/2, -11/2, h-4]) cube([11, 11, 5]);
            }
        }
    }
}

module hexagon(size, height) {
  boxWidth = size/1.75;
  for (r = [-60, 0, 60]) rotate([0,0,r]) cube([boxWidth, size, height], true);
}

nut_dia = 5.3;
nut_depth=5.5;
nut_off_y = 80/2;
nut_off_x = 20;

module nut_holder(
    a = 30,
    b = 15,
    c = 10,
    d = 5.0,
    e = 4.0,
    ) {
        
    difference() {
        union() {
            translate([-b/2, -(a-b)/2, 0]) cube([b, a-b, c]);
            translate([0, -(a-b)/2, 0]) cylinder(d=b, h=c);
            translate([0, (a-b)/2, 0]) cylinder(d=b, h=c);
        }
        translate([0,  a/2 - d, -eps]) {
            translate([0, 0, c-nut_depth+2*eps]) cylinder(d=nut_dia, h=nut_depth);
            cylinder(d=e, h=c+2*eps);
        }
        translate([0, -a/2 + d, -eps]) {
            translate([0, 0, c-nut_depth+2*eps]) cylinder(d=nut_dia, h=nut_depth);
            cylinder(d=e, h=c+2*eps);
        }
    }
}

module base($fn=25, cw=90, sw=15, sh=15, strut_spacing=30, clamp_dist=90) {
    d = 30;
    clamp(l=cw, h=sh);
    translate([-d, 0, 0]) nut_holder();
    translate([d, 0, 0]) nut_holder();
    /*difference() {
        translate([0, -clamp_dist/2, 0]) union() {
    */
            /*
            translate([0,    0, 0]) clamp(l=cw, h=sh);
            translate([-strut_spacing/2-sw, 0, 0]) cube([sw, clamp_dist, sh]);
            translate([ strut_spacing/2, 0, 0]) cube([sw, clamp_dist, sh]);
            */
        //}
        /*
        translate([nut_off_x, nut_off_y, -eps]) cylinder(d=nut_dia, h=nut_depth+eps);
        translate([nut_off_x, -nut_off_y, -eps]) cylinder(d=nut_dia, h=nut_depth+eps);
        translate([-nut_off_x, nut_off_y, -eps]) cylinder(d=nut_dia, h=nut_depth+eps);
        translate([-nut_off_x, -nut_off_y, -eps]) cylinder(d=nut_dia, h=nut_depth+eps);
        */
    //}
}

base();