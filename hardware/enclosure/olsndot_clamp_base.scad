eps = 0.01;

module clamp(l=90, w=20, h=10, d1=25, d2=35, o=10, hdia1=5, hdia2=10, hd=1, nw=7.8, nd=3, notch_sf=0.75, notch_d=2, notch_a=10, notch_o=1, edge_a=5, edge_d=12) {
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
                cylinder(d=hdia1, h=l);
                cylinder(d=hdia2, h=hd);
                translate([0, 0, h-nd/2]) hexagon(nw, nd+2*eps);
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

module base($fn=25, cw=90, sw=15, sh=10, strut_spacing=30, clamp_dist=90) {
    difference() {
        translate([0, -clamp_dist/2, 0]) union() {
            translate([0,  clamp_dist, 0]) clamp(l=cw, h=sh);
            /*
            translate([0,    0, 0]) clamp(l=cw, h=sh);
            translate([-strut_spacing/2-sw, 0, 0]) cube([sw, clamp_dist, sh]);
            translate([ strut_spacing/2, 0, 0]) cube([sw, clamp_dist, sh]);
            */
        }
        /*
        translate([nut_off_x, nut_off_y, -eps]) cylinder(d=nut_dia, h=nut_depth+eps);
        translate([nut_off_x, -nut_off_y, -eps]) cylinder(d=nut_dia, h=nut_depth+eps);
        translate([-nut_off_x, nut_off_y, -eps]) cylinder(d=nut_dia, h=nut_depth+eps);
        translate([-nut_off_x, -nut_off_y, -eps]) cylinder(d=nut_dia, h=nut_depth+eps);
        */
    }
}

base();