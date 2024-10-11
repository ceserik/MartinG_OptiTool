
%% compares singltrek and twintrek if they are equal change car.track to 0 an they should be equivalent


twinu = [ 
    30
    -3
    0
    -10
    -10
    0
    ];
u = [
    30
    -3
    -10
    -10
    0
];

z = [ 
    -10
    -10
    -10
    -10
    -10
    -10
    -10
    ];

n = 100000;
field = zeros(n,1);
x=0;
for i = 1:n
    z = rand(7,1)*100 - rand(7,1)*100;
    twinu = rand(6,1)*100 - rand(6,1)*100;
    singu = [
        twinu(1);
        twinu(2);
        twinu(3)+twinu(4) ;
        twinu(5)+twinu(6);
        0 ];
    twin_dz = twinRaceCar_time_ODE(z,twinu,car);
    dz = raceCar_time_ODE(z,singu,car);
x = x + sum(abs(dz-twin_dz));
field(i) = sum(abs(dz-twin_dz));
%disp(sum(abs(dz)-abs(twin_dz)))
end
x
max(field)

