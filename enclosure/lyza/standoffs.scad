inner_dia = 3.0;
outer_dia = inner_dia + 2*1.5;
height    = 18.0;
eps       = 0.1;

module standoff() {
    difference() {
        cylinder(h=height, d=outer_dia);
        translate([0, 0, -eps]) cylinder(h=height+2*eps, d=inner_dia);
    }
}

module standoff_group() {
    standoff();
}

standoff_group($fn=25);