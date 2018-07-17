
/* A small, arbitrary unit to guard against rounding errors */
eps = 0.1;
alot = 1000;

/* Base PCB size */
pcb_width = 50;
pcb_height = 70;
/* Extra space around PCB */
pcb_extra = 1;

/* Offset of screw hole centers from left/right/top/bottom side of board */
screw_offx_l = 5;
screw_offx_r = 5;
screw_offx_t = 5;
screw_offx_b = 5;

/* Thickness of bottom, walls, lid (middle sections) */
bottom_thickness = 2;
wall_thickness = 2;
lid_thickness = 2;

/* Outside corner radius */
corner_radius_wall = 3;
corner_radius_lid = 2;

screw_base_dia = 8;
screw_base_h = 5;
screw_nut_dia = 5.4;
screw_nut_h = 6;
screw_hole_dia = 3.2;
screw_hole_h = 6;
board_thickness = 1.6;
height_above_board = 18;
height_lid_inner = 2;
lid_lip_depth = 2;
lid_lip_width = 2;
lid_lip_extra = 1;

clamp_tube_dia = screw_base_dia;
clamp_tube_h = 0; //height_above_board+height_lid_inner;
clamp_tube_base_extra = 2;

screw_head_hole_dia = 6.5;
screw_head_hole_depth = 3;

module rounded_rect(w, h, r) {
    hull(){
        translate([w/2-r, h/2-r]) circle(r);
        translate([-(w/2-r), h/2-r]) circle(r);
        translate([w/2-r, -(h/2-r)]) circle(r);
        translate([-(w/2-r), -(h/2-r)]) circle(r);
    }
}

module shell(thickness, inner_height, inner_extra, radius_horiz) {
    d = thickness + inner_height;
    rh=radius_horiz;
    difference(){
        r = corner_radius_wall;
        w = pcb_width+pcb_extra+2*wall_thickness;
        h = pcb_height+pcb_extra+2*wall_thickness;
        hull() {
            translate([0, 0, rh]) linear_extrude(height=d-rh) rounded_rect(w, h, r);
            linear_extrude(height=eps) rounded_rect(w-2*rh, h-2*rh, r);
        }
        
        iw = pcb_width+inner_extra;
        ih = pcb_height+inner_extra;
        translate([-iw/2, -ih/2, bottom_thickness]) cube([iw, ih, h]);
    }
}

module single_screw_base() {
    translate([0, 0, bottom_thickness-eps]) linear_extrude(height=screw_base_h+eps) {
        circle(d=screw_base_dia);
        translate([-screw_base_dia/2, 0]) square([alot, alot]);
        translate([0, -screw_base_dia/2]) square([alot, alot]);
    }
}

w = pcb_width;
h = pcb_height;
e = pcb_extra;
l = screw_offx_l;
r = screw_offx_r;
t = screw_offx_t;
b = screw_offx_b;
s1_pos = [w/2-l, h/2-b];
s2_pos = [-w/2+r, -h/2+t];
s3_pos = [w/2-l, -h/2+t];
s4_pos = [-w/2+r, h/2-b];
module screw_bases() {
    intersection() {
        union() {
            translate(s1_pos) single_screw_base();
            translate(s2_pos) mirror([1,1,0]) single_screw_base();
            translate(s3_pos) mirror([0,1,0]) single_screw_base();
            translate(s4_pos) mirror([1,0,0]) single_screw_base();
        }
        cube([w+e+eps, h+e+eps, alot], center=true);
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

conn_1_y = 12.0;
conn_1_l = 20.0;
conn_1_h = 7;
conn_2_y =  conn_1_y+conn_1_l+6.0;
conn_2_l = conn_1_l;
conn_2_h = conn_1_h;
conn_pwr_y = 12.5;
conn_pwr_l = 10.0;
conn_pwr_h = 10;
conn_extra = 1;
module conn_holes() {
    e = conn_extra;
    translate([0, -e/2, -e/2])
    translate([0, 0, bottom_thickness+screw_base_h+board_thickness])
    translate([-pcb_width/2, -pcb_height/2]) {
        translate([-alot, conn_1_y]) cube([alot, conn_1_l+e, conn_1_h+e]);
        translate([-alot, conn_2_y]) cube([alot, conn_2_l+e, conn_2_h+e]);
        translate([0, conn_pwr_y]) cube([alot, conn_pwr_l+e, conn_pwr_h+e]);
    }
}

module base() {
    difference() {
        union() {
            screw_bases();
            shell(bottom_thickness,
                screw_base_h + board_thickness + height_above_board,
                pcb_extra,
                radius_horiz=0);
        }
        screw_holes();
        conn_holes();
    }
}

module single_clamp_stud_part(liph, clamp_tube_h, extra) {
    translate([0, 0, lid_thickness-eps]) union() {
        cylinder(h=clamp_tube_h, d=clamp_tube_dia+extra);
        linear_extrude(height=liph+eps) {
            translate([-clamp_tube_dia/2-extra/2, 0]) square([alot, alot]);
            translate([0, -clamp_tube_dia/2-extra/2]) square([alot, alot]);
        }
    }
}

module single_clamp_stud(liph) {
    e = clamp_tube_base_extra;
    single_clamp_stud_part(liph, liph+eps, e);
    single_clamp_stud_part(liph, clamp_tube_h, 0);
}

module lid_screw_hole() {
    cylinder(h=alot, d=screw_hole_dia);
    cylinder(h=screw_head_hole_depth+eps, d=screw_head_hole_dia);
}

module lid() {
    e = lid_lip_extra;
    iw = pcb_width+pcb_extra-e+eps;
    ih = pcb_height+pcb_extra-e+eps;
    liph = lid_lip_depth+height_lid_inner;
    difference() {
        union() {
            shell(lid_thickness,
                height_lid_inner,
                inner_extra=-eps,
                radius_horiz=corner_radius_lid);
            difference() {
                translate([-iw/2, -ih/2, lid_thickness-eps]) cube([iw, ih, liph+eps]);
                // FIXME translate([-iw/2+e/2, -ih/2+e/2, lid_thickness-eps]) cube([iw-e, ih-e, liph+2*eps]);
            }
            intersection() {
                translate([-iw/2, -ih/2, lid_thickness-eps]) cube([iw, ih, alot]);
                union() {
                    translate(s1_pos) single_clamp_stud(liph);
                    translate(s2_pos) mirror([1,1,0]) single_clamp_stud(liph);
                    translate(s3_pos) mirror([0,1,0]) single_clamp_stud(liph);
                    translate(s4_pos) mirror([1,0,0]) single_clamp_stud(liph);
                }
            }
        }
        translate([0, 0, -eps]) {
            translate(s1_pos) lid_screw_hole();
            translate(s2_pos) lid_screw_hole();
            translate(s3_pos) lid_screw_hole();
            translate(s4_pos) lid_screw_hole();
        }
    }
}

module enclosure() {


    intersection() {
        //translate([-pcb_width, -40, 0]) cube (35, 20, 20);
        translate([-pcb_width-15, 0, 0]) base();
    };

/*
    intersection() {
        //translate([-alot/2, -10, -alot/2]) cube ([alot, 20, alot]);
        union () {
            render()    lid();
            render() translate([0,0,eps]) mirror([0,0,1]) intersection() {
                translate([-alot/2, -alot/2, 0]) cube([alot, alot, alot]);
                translate([0,0,-0.2]) scale([0.3, 0.3, 0.04]) surface("depthmap_03.png", center=true);
            };
        };
    };
*/
}

enclosure($fn=25);