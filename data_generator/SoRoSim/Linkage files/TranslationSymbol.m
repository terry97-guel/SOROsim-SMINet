function Pos = TranslationSymbol

x=0.8;
y=0.2;
x=[x;1];
y=[y;0];
x=[x;0.8];
y=[y;-0.2];
x=[x;1];
y=[y;0];

x=[x;-1];
y=[y;0];
x=[x;-0.8];
y=[y;0.2];
x=[x;-1];
y=[y;0];
x=[x;-0.8];
y=[y;-0.2];

Pos=[x y zeros(length(x),1)];


end

