include <carriage.scad>

base_width = 60;

axel_position = 130;
rod_tolerance = 0.25;

head_height = 10;
head_width = 40;
head_depth = 60;

head_z_offset = 15;
head_x_offset = -5;
neck_offset = 0;

neck_width = head_width;
neck_depth = 45;
neck_height = axel_position - head_z_offset - head_height/2 -base_height/2;

hole_depth = 5;

$fa = 1;
$fs = 0.2;


//the neck

translate([0,0,base_height])
difference() {
    hull() {
        
        cube([base_width, base_depth, epsilon], center=true);
        
        translate([-base_width/2+head_x_offset+neck_offset,-neck_depth/2,neck_height-epsilon/2])
            cube([neck_width,neck_depth,epsilon]);
        
    }
    
    
    adj = head_x_offset+neck_offset;
    angle = atan(adj / neck_height);
    
    echo(angle);
    
    translate([-base_width/2,0,0])
        rotate([0,angle,0])
        translate([hole_depth,0,0])
        rotate([0,-90,0])
        linear_extrude(height=M,center=false)
        polygon([
            [0,-base_depth/6], [0,base_depth/6], [neck_height/cos(angle),0]
        ]);
    
    
    adj2 = base_width - (head_x_offset+neck_offset+neck_width);
    angle2 = -atan(adj2/neck_height);

    translate([base_width/2,0,0])
        rotate([0,angle2,0])
        translate([-hole_depth,0,0])
        scale([-1,1,1])
        rotate([0,-90,0])
        linear_extrude(height=M,center=false)
        polygon([
            [0,-base_depth/6], [0,base_depth/6], [neck_height/cos(angle2),0]
        ]);
    
    
    adj3 = base_depth/2 - neck_depth/2;
    angle3 = atan(adj3/neck_height);
    
    echo(angle3);
    
    mirror_y()
        translate([0,base_depth/2,0])
        rotate([angle3,0,0])
        translate([0,-hole_depth,0])
        scale([1,-1,1])
        rotate([90,0,0])
        linear_extrude(height=M,center=false)
        polygon([
            [-base_width/6,0], 
            [base_width/6,0], 
            [-base_width/2+head_x_offset+neck_offset+neck_width/2,neck_height/cos(angle3)]
        ]);
        
}

//the head


translate([-base_width/2+head_x_offset,0,base_height+neck_height])
difference() {
    hull() {
        translate([neck_offset,-neck_depth/2,-epsilon/2])
            cube([neck_width,neck_depth,epsilon]);

        translate([0,-head_depth/2,head_z_offset])
            cube([head_width,head_depth,head_height]);
        
        translate([neck_offset,-neck_depth/2,head_z_offset*2+head_height+epsilon/2])
            cube([neck_width,neck_depth,epsilon]);
        
        
        translate([head_width+80,0,head_z_offset+head_height/2])
        cube([epsilon,epsilon,epsilon],center=true);
        
        translate([-80,0,head_z_offset+head_height/2])
        cube([epsilon,epsilon,epsilon],center=true);
    }
    
    side = sqrt(pow(head_z_offset,2)+pow(head_depth-neck_depth,2)/4);
    
    //the cutoffs for the design thing
    
    translate([M/2+head_width+30,0,0])
    cube([M,M,M],center=true);
    
    translate([-M/2-10,0,0])
    cube([M,M,M],center=true);
    
    
    //the plus sign hole thing for the design or something
    translate([head_width-epsilon,-M/2,head_z_offset])
        cube([M,M,head_height]);
    
    translate([head_width-epsilon,-neck_depth/2+side,-M/2])
        cube([M,neck_depth-side*2,M]);
        
    translate([-M-epsilon,-M/2,head_z_offset])
        cube([M,M,head_height]);
    
    translate([-M+-epsilon,-neck_depth/2+side,-M/2])
        cube([M,neck_depth-side*2,M]);

    
    //the smooth rod
    translate([0,0,head_height/2+head_z_offset])
        rotate([0,90,0])
        cylinder(d=smooth_rod_diameter+rod_tolerance*2,h=M,center=true);
    
    //the indent for the flange-bearings
    translate([flange_bearing_thickness-epsilon,0,head_height/2+head_z_offset])
        rotate([0,-90,0])
        rotate([0,0,90])
        linear_extrude(height=M)
        flange_bearing();
    
    translate([head_width-flange_bearing_thickness+epsilon,0,head_height/2+head_z_offset])
        rotate([0,90,0])
        rotate([0,0,90])
        linear_extrude(height=M)
        flange_bearing();
        
    //the bolt holes
    mirror_y()
        translate([
            0,
            flange_bearing_max_width/2 - flange_bearing_bolt_diameter/2 - flange_bearing_bolt_offset,
            head_z_offset+head_height/2
         ])
        rotate([0,90,0])
        cylinder(d=flange_bearing_bolt_diameter,h=M,center=true);
     
    
}
