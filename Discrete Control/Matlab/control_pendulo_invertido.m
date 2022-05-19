%Control de Velocidad de Motor de DC
%Link: https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlStateSpace
%Autor: Trinidad Burs
%Fecha: 21-04-2021

clc
clear
close
%% MODELIZACION
%Para ver que representan las variables se recomienda ver la modelizacion en la pagina
M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = eye(size(A));%En la pagina era:[1 0 0 0; 0 0 1 0];
D = zeros(size(B));%En la pagina era: [0;0];
C_original = [1 0 0 0;0 0 1 0];
D_original = [0;0];
%Cambio los valores de C y D para obtener todas las salidas. Sera util en
%la simulacion para comparar como esta funcionando el observador y si esta
%funcionando correctamente
%% VERIFICACION DE CONTROLABILIDAD
M_controlabilidad=ctrb(A,B);
if (size(A,1)==rank(M_controlabilidad)) %El order de A tiene que coincidir con el numero de filas de la matriz de controlabilidad
    disp("El sistema es controlable")
else
    error('El sistema no es controlable!')
end

%% VERIFICACION DE OBSERVABILIDAD
M_observabilidad=ctrb(A,C);
if (size(A,1)==rank(M_observabilidad)) %El order de A tiene que coincidir con el numero de filas de la matriz de controlabilidad
    disp("El sistema es observable")
else
    error('El sistema no es observable!')
end
%% SISTEMA ANALOGICO
sys_ol = ss(A,B,C,D); %Este es el espacio de estado en open loop
sys_ol_order = order(sys_ol)  %Orden del sistema --> rango de la matriz A
sys_ol_rank = rank(ctrb(A,B)) %Tiene que ver con la controlabilidad--> ctrb(A,B): matriz de controlabilidad

%% CALCULO DE K EN FUNCION DE LOS POLOS DEL SISTEMA
%Se trato de conseguir las especificaciones del ejercicio con el criterio
%de ubicacion de polos de itae
wo=3.5;
p = [(-0.424+j*1.263) (-0.424-j*1.263) (-0.626+j*0.414) (-0.626-j*0.414)]*wo; %POLOS SEGUN ITAE
%p_bessel = [(-0.6573+j*0.8302) (-0.6573-j*0.8302) (-0.9047+j*0.2711) (-0.9047-j*0.2711)]*wo;
Kc = place(A,B,p);  %Kc es el K de realimentacion, lo busco de tal manera que los polos me caigan en p

%% CIERRO EL LAZO Y VERIFICO
A_cl=A-B*Kc;
sys_cl=ss(A_cl,B,C,D);
p = pole (sys_cl)
p_aux = eig(A_cl) %Coinciden con p!! Si no coinciden algo hice mal!!OJOOOOOO
step(sys_cl)

%% VERIFICACION DE ESTABILIDAD A LAZO ABIERTO
%EL SISTEMA SERA INESTABLE A LAZO ABIERTO!!!! OJO!!!!
%Observamos la primera variable, aumentara infinitamente, luego todo el sistema sera inestable
aux=stepinfo(sys_ol); %If sys is unstable, then all step-response characteristics are NaN, except for Peak and PeakTime, which are Inf.
if (isequaln(aux(1).RiseTime,NaN))
    disp('El sistema es INESTABLE a lazo abierto')
else
    disp('El sistema no es inestable a lazo abierto')
end

%Otra forma de verificarlo es ver si alguno de los polos del sistema
%abierto tiene algun polo en el semieje negativo. De tenerlo sera inestable
if (any(eig(A)>0)==1)
    disp('El sistema es INESTABLE a lazo abierto')
else
    disp('El sistema no es inestable a lazo abierto')
end
%% VERIFICACION DE ESTABILIDAD A LAZO CERRADO
aux=stepinfo(sys_cl);
if (isequaln(aux(1).RiseTime,NaN))
    disp('El sistema es INESTABLE a lazo cerrado')
else
    disp('El sistema no es inestable a lazo cerrado')
end
%% ESCALO LA ENTRADA
%Por como funciona rscale si meto las C y D que yo defini no concuerdan las
%dimensiones de las matrices cuando uso rscale. Lo mismo sucede con las
%matreices C y D originales.
%ESTO ES PORQUE CUANDO ESCALO SOLO PUEDO DETERMINAR EL VALOR FINAL DE UNA
%VARIBALE, EL RESTO SE ADAPTARA A LA CONSTANTE DEL MULTIPLICADOR QUE SE ELIJA PARA LA OTRA
C_modificada = [1 0 0 0];
D_modificada = [0];
sys_ol_aux = ss(A,B,C_modificada,D_modificada); %Este es el espacio de estado de open loop
Nbar = rscale_new(sys_ol_aux,Kc); %Escalo la salida a la entrada. Lo realice usando los valores de C y D propuestos en el link ya que con C=eye(...) y D=zeros(...) no podia calcular la inv de rscale
%% DISCRETIZO EL ESPACIO DE ESTADOS (este no tendra mucha utilidad ya que para simular se debe usar la version "fast")
aux1=stepinfo(sys_cl); %quiero ver la velocidad de respusta del sistema a lazo abierto (del sistema original)
Ts=aux1(1).RiseTime/20;%aux1(:,1).RiseTime/20;
[Ad,Bd,Cd,Dd] = c2dm(A,B,C,D,Ts,'zoh'); %Paso el espacio de estados a discreto

%CALCULO DE Kd EN FUNCION DE LOS POLOS DEL SISTEMA DISCRETO
pd= exp(p*Ts);  % mapeo los polos a discreto con tiempo muestreo Ts
Kd = place(Ad,Bd,pd);

%CIERRO EL LAZO
A_cl_d=Ad-Bd*Kd;
sys_cl_d=ss(A_cl_d,Bd,Cd,Dd,Ts);

%ESCALO
settlingValue_d=dcgain(sys_cl_d);
Amplitud_Entrada=1;
Nbard=Amplitud_Entrada/settlingValue_d(1); %Escalo en funcion de la posicion
%% OBSERVADOR
%Analogico
po = p*4; % Los fijo al menos 4 veces mas rapidos que los polos del sistema
L=(place(A',C',po))';
Ao=A-L*C;
Bo=[B,L]; %Artilugio matematico para poder implementar el observador en el simulink! SUPER IMPORANTE!
Co=eye(size(A));
Do=zeros(size(Bo));

%Discreto
Tso=Ts/4;
[Aod,Bod,Cod,Dod] = c2dm(Ao,Bo,Co,Do,Tso,'zoh');
%% SISTEMA ORIGINAL A DISCRETO PARA SIMULAR EN MICRO ("FAST")
%Se va a simular con Tso para que el observador pueda segurilo mejor
%Si tenemos que implemenatr este sistema en un micro (para una simulacion)
%tenemos que usar este
[Ad_fast,Bd_fast,Cd_fast,Dd_fast] = c2dm(A,B,C,D,Tso,'zoh');

%CALCULO DE Kd EN FUNCION DE LOS POLOS DEL SISTEMA DISCRETO
pd_fast=exp(p*Tso)  % mapeo los polos a discreto con tiempo muestreo Ts
Kd_fast = place(Ad_fast,Bd_fast,pd_fast)

%CIERRO EL LAZO
A_cl_d_fast=Ad_fast-Bd_fast*Kd_fast;
sys_cl_d_fast=ss(A_cl_d_fast,Bd_fast,Cd_fast,Dd_fast,Tso);

%ESCALO
settlingValue_d_fast=dcgain(sys_cl_d_fast);
Amplitud_Entrada=1;
Nbard_fast=Amplitud_Entrada/settlingValue_d_fast(1); %Escalo en funcion de la posicion
