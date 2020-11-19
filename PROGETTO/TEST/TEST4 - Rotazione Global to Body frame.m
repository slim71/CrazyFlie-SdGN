syms a b c

%Costruiamo la matrice simbolica di rotazione come composizione delle tre
%matrici: 

R_x = [ 1       0           0 ; ...
        0       cos(a)      -sin(a); ...
        0       sin(a)      cos(a)];

R_y = [ cos(b)  0      sin(b) ; ...
        0       1      0; ...
        -sin(b) 0      cos(b)];
   
    
R_z = [ cos(c)  -sin(c)     0 ; ...
        sin(c)  cos(c)      0; ...
        0       0           1];

%Il tracker fornisce roll (a) pitch (b) yaw (c) del drone rispetto al
%sistema di riferimento fisso Vicon "V". Questo sistema ha il suo asse X
%rivolto verso l'armadio, asse Y verso la finestra e asse Z verso l'alto. 

%In generale abbiamo un sistema V e un sistema "D" (che costituisce il sistema body del drone).
%Questo sistema "D" solidale al drone ha il suo asse X rivolto da poppa a
%prua, asse Y in modo da avere una terna destrorsa con asse Z rivolto verso
%l'alto. 






%Per portare un vettore espresso in  V ad uno espresso in D dobbiamo in generale scrivere la  D = R_dv*V 
%in cui la R_dv esprime come passare da D a V (Per portare in realtà V in D). 

%La R_xyz per noi esprime come passare da V a D dunque ciò che serve a noi
%è la sua trasposta. 

R_xyz = R_x*R_y*R_z;

R_zyx = R_xyz'; %R_dv simbolica

%Partiamo dal drone posizionato secondo il "V" con questi valori 
% D_POS_V:      x y z = 1 1 0   [m]
% D_ORIENT:     a b c = 0 0 90  [deg]

%Otteniamo la R_dv numerica:
R_prova1 = subs(R_zyx,a,deg2rad(0));
R_prova2 = subs(R_prova1,b,deg2rad(0));
R_dv = subs(R_prova2,c,deg2rad(90));
determinante = eval(det(R_prova3)) %Controlliamo che il determinante sia 1

%Costruiamo la matrice (e il vettore) omogenea di trasformazione:
D_POS_V = [1 1 0]'
D_POS_D = -eval(R_dv*D_POS_V)
R_dv_omogenea = [eval(R_dv) D_POS_D; 0 0 0 1];
D_POS_V_omogeneo = [D_POS_V' 1]';  %Saranno le due informazioni da salvare all'inizio dell'esperimento e da riutilizzare per tutto il suo seguito

%Se i conti sono giusti questo vettore deve essere il vettore nullo. 
D_POS_D_omogeneo = R_dv_omogenea*D_POS_V_omogeneo
%A questo punto siamo sicuri di aver fatto bene il passaggio tra i sistemi "V" e "D"









%Supponiamo di voler spostare il drone dal suo punto di partenza alla nuova
%posizione espressa nel sistema "V" (Sarebbe la nuova posizione della Wand cosi 
%come ci viene fornita dal Vicon)
%W_POS_V:  x y z = 1 -1.5 1 [m]
W_POS_V_omogeneo = [1 -1.5 1 1]';

%Il comando che diamo realmente al drone sarà la W_POS (ottenuta in "V")
%convertita nel sistema "D":
W_POS_D_omogeneo = R_dv_omogenea*W_POS_V_omogeneo

%Il risultato finale deve essere il seguente:
%W_POS_D: x y z = -2.5 0 1 [m]
%che rappresenta la nuova posizione della Wand espressa nel sistema "D",
%ovvero il punto in cui il drone deve andare "secondo il suo sistema di
%riferimento". 



