
/* A small, arbitrary unit to guard against rounding errors */
eps = 0.1;
alot = 1000;

/* Base PCB size */
pcb_width = 90;
pcb_height = 50;

bottom_thickness = 1.5;

screw_base_dia = 8;
screw_base_h = 5;
screw_nut_dia = 5.4;
screw_nut_h = 6;
screw_hole_dia = 3.5;
screw_hole_h = 6;

module rounded_rect(w, h, r) {
    hull(){
        translate([w/2-r, h/2-r]) circle(r);
        translate([-(w/2-r), h/2-r]) circle(r);
        translate([w/2-r, -(h/2-r)]) circle(r);
        translate([-(w/2-r), -(h/2-r)]) circle(r);
    }
}

module single_screw_base() {
    translate([0, 0, bottom_thickness-eps]) linear_extrude(height=screw_base_h+eps) {
        circle(d=screw_base_dia);
        translate([-screw_base_dia/2, 0]) square([alot, alot]);
        translate([0, -screw_base_dia/2]) square([alot, alot]);
    }
}

w = pcb_width+10;
h = pcb_height;
l = 10;
r = 10;
t = 5;
b = 5;
s1_pos = [ w/2-l,  h/2-b];
s2_pos = [-w/2+r, -h/2+t];
s3_pos = [ w/2-l, -h/2+t];
s4_pos = [-w/2+r,  h/2-b];
module screw_bases() {
    intersection() {
        union() {
            translate(s1_pos) single_screw_base();
            translate(s2_pos) mirror([1,1,0]) single_screw_base();
            translate(s3_pos) mirror([0,1,0]) single_screw_base();
            translate(s4_pos) mirror([1,0,0]) single_screw_base();
        }
        cube([w+eps, h+eps, alot], center=true);
    }
}


module single_screw_hole() {
    translate([0, 0, bottom_thickness+screw_base_h+eps])
    mirror([0,0,1])
    union() {
        cylinder(h=screw_nut_h+eps, d=screw_nut_dia);
        cylinder(h=screw_hole_h+eps, d=screw_hole_dia);
    }
}

module screw_holes() {
    translate(s1_pos) single_screw_hole();
    translate(s2_pos) mirror([1,1,0]) single_screw_hole();
    translate(s3_pos) mirror([0,1,0]) single_screw_hole();
    translate(s4_pos) mirror([1,0,0]) single_screw_hole();
}

tab_w = 20;
tab_h = 30;
tab_thickness = 3;
tab_hole_d = 9;
module screw_tab() {
    difference() {
        translate([-tab_h/2, -eps, 0]) cube([tab_h, tab_w+eps, tab_thickness]);
        rotate([0, 0, -50]) translate([-alot/2, tab_h*0.6, -eps]) cube([alot, alot, tab_h+2*eps]);
        rotate([0, 0, 50]) translate([-alot/2, tab_h*0.6, -eps]) cube([alot, alot, tab_thickness+2*eps]);
        translate([0, tab_w/2, -eps]) cylinder(d=tab_hole_d, h=tab_thickness+eps*2);
    }
}

wall_height = bottom_thickness + 3;
wall_thickness = 1.5;

cutout_w = 25;
cutout_h = 70;

module carrier() {
    union() {
        difference() {
            translate([0, 0, wall_height/2]) cube([w, h, wall_height], center=true);
            translate([0, 0, bottom_thickness + alot/2]) cube([w-wall_thickness*2+eps, h-wall_thickness*2+eps, alot], center=true);
            cube([cutout_h, cutout_w, alot], center=true);
        }
        difference() {
            screw_bases();
            screw_holes();
        }
        translate([0, h/2, 0]) screw_tab();
        mirror([0,1,0]) translate([0, h/2, 0]) screw_tab();
    }
}

carrier($fn=25);