$fn=40;

L1 = 15;
L2 = 130;
L3 = 130;
L4 = 70;

x = 100.0;
y = 100.0 + 10.0;
z = 110.0;
p = -60.0*$t+30.0;
r = 0.0;
oc = 0.0;

v5 = [x,y,z];


difference(){
	sphere(1);
	difference(){
		sphere(260);
		translate([0,0,-300])cube([600,600,600],true);
	}
}

L5 = ((20.4)*sin(((255.0*oc)/3.269))+(80.0));
phi1 = atan2(y,x);

v1 = [L1*cos(phi1),L1*sin(phi1),0];

L45 = L4+L5;

xy3 = L45*cos(p);
v3 = [x - xy3*cos(phi1),y - xy3*sin(phi1),z - L45*sin(p)];

L23 = sqrt((v3[0]-v1[0])*(v3[0]-v1[0]) + (v3[1]-v1[1])*(v3[1]-v1[1]) + (v3[2]-v1[2])*(v3[2]-v1[2]));



xy = sqrt(x*x + y*y);
xy1=xy-L1- cos(p)*L45;
z1 = z-sin(p)*L45;

L13=sqrt(z1*z1+xy1*xy1);

phi2 = acos((L2*L2 + L23*L23 - L3*L3)/(2*L2*L23))+atan2(z1,xy1);
phi3 = acos((L2*L2 + L3*L3 - L23*L23)/(2*L2*L3));


phi4 = p-(phi2+phi3)+180;

echo("phi1: ",phi1);
echo("phi2: ",phi2);
echo("phi3: ",phi3);

echo("phi4: ",phi4);

//phi1 = 0;
//phi2 = 69;
//phi3 = 180;
//phi4 = 180;
//rot = 0;
//oc = 1;

//translate([0,0,-42])servo_base_mount();

sphere(5);
translate(v1)sphere(5);
translate(v3)sphere(5);
translate(v5)sphere(5);

rotate([0,0,phi1])robot_arm_base();

module robot_arm_base(){
	rotate([0,90,0])cylinder(r=2,h=L1);
	translate([L1,0,0])rotate([0,-phi2,0])rotate([0,90,0])lower_arm();
}

module lower_arm(){
	color([0.0,0.5,0.0])cylinder(r=2,h=L2);
	translate([0,0,L2])rotate([0,-phi3,0])rotate([-180,0,0])upper_arm();
}

module upper_arm(){
	color([0.0,0.0,0.5])cylinder(r=2,h=L3);
	translate([0,0,L3])for_arm();
}

module for_arm(){
	rotate([0,phi4,0])rotate([0,0,0])wrist();
}

module wrist(){
	color([0.8,0.8,0.8])cylinder(r=2,h=L4);
	color([0.2,0.2,0.2])translate([0,0,70])rotate([0,0,r])claw(oc);
}

module claw(oc){
	cylinder(r=2,h=L5);
	translate([-L1/2,0,L5])rotate([0,90,0])cylinder(r=2,h=L1);
}

module base_holder(){
	union(){
	difference(){	
		difference(){
			translate([0,0,-45/2 + 2])cube([82,82,45],true);
			union(){
				translate([0,0,-43/2])cube([78,78,45],true);
				translate([0,0,-43/2])cube([5,90,35],true);
				translate([10,0,-43/2])cube([5,90,35],true);
				translate([20,0,-43/2])cube([5,90,35],true);
				translate([30,0,-43/2])cube([5,90,35],true);
				translate([-30,0,-43/2])cube([5,90,35],true);
				translate([-10,0,-43/2])cube([5,90,35],true);
				translate([-20,0,-43/2])cube([5,90,35],true);
				translate([-30,0,-43/2])cube([5,90,35],true);
				translate([0,10,-43/2])cube([90,5,35],true);
				translate([0,20,-43/2])cube([90,5,35],true);
				translate([0,30,-43/2])cube([90,5,35],true);
				translate([0,-10,-43/2])cube([90,5,35],true);
				translate([0,-20,-43/2])cube([90,5,35],true);
				translate([0,-30,-43/2])cube([90,5,35],true);
				translate([0,0,-43/2])cube([90,5,35],true);
			}
		}
		union(){			
			translate([-33,-33,-5])cylinder(r=1.5,h=10);
			translate([33,-33,-5])cylinder(r=1.5,h=10);
			translate([33,33,-5])cylinder(r=1.5,h=10);
			translate([-33,33,-5])cylinder(r=1.5,h=10);
			translate([0,0,-5])cylinder(r=15,h=10);
		}
	}
	difference(){
		union(){
			translate([-21,-15,-9])cube([10,30,11]);
			translate([31,-15,-9])cube([10,30,11]);
		}
		union(){
			translate([-14,5,-20])cylinder(r=1.25,h = 18);
			translate([-14,-5,-20])cylinder(r=1.25,h = 18);
			translate([35,5,-20])cylinder(r=1.25,h = 18);
			translate([35,-5,-20])cylinder(r=1.25,h = 18);
		}
	}
}
}

module servo_base_mount(){
	//translate([0, 0, 42])rotate([0,0,0])Servo(1,1);
	difference(){
		union(){
			translate([-20,-15,0])cube([60,30,30]);
			translate([-20,-25,0])cube([10,50,5]);
			translate([30,-25,0])cube([10,50,5]);
		}	
		union(){
			translate([22,-4,-1])cube([20,8,10]);
			translate([-15,-20,-1])cylinder(r=2,h=7);
			translate([-15,20,-1])cylinder(r=2,h=7);
			translate([35,-20,-1])cylinder(r=2,h=7);
			translate([35,20,-1])cylinder(r=2,h=7);
			translate([-11.5,-11,-1])cube([44,22,40]);
			translate([-14,5,20])cylinder(r=1.25,h = 50);
			translate([-14,-5,20])cylinder(r=1.25,h = 50);
			translate([35,5,20])cylinder(r=1.25,h = 50);
			translate([35,-5,20])cylinder(r=1.25,h = 50);
		}
	}
}

module Servo(s,h)
{
	scale([s,s,s]) translate([-10, -9.5, -41.5]) union() {
		color([0.2,0.2,0.2]) union() {
			cube([40,19,40]);
			difference() {
				translate([-7.5, 0, 29]) cube([55, 19, 3]);
				translate([-5, 4.5, 28]) cylinder(r=2,h=5);
				translate([-5, 14.5, 28]) cylinder(r=2,h=5);
				translate([45, 4.5, 28]) cylinder(r=2,h=5);
				translate([45, 14.5, 28]) cylinder(r=2,h=5);
			}
			translate([10, 9.5, 40])cylinder(r=6.5,h=1.5);
		}
		color([1,215/255,0]) translate([10, 9.5, 41.5])cylinder(r=3,h=4.5);
		if (h == 1)
		{
			union() {
		   		translate([10, 9.5, 43]) color([0.2,0.2,0.2]) cylinder(r=3.5,h=6);
				translate([10, 9.5, 48]) color([0.2,0.2,0.2]) cylinder(r=10,h=2);	
			}			
		} 
	}
	
}