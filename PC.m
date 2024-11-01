% Paso 1: Leer los datos desde el archivo Excel
datos = xlsread('TL.csv', 1, 'A1:C348');  % Lee datos de las columnas A, B, y C hasta la fila 348

% Paso 2: Separar las columnas
x = datos(:, 1);  % Primera columna (A) completa para el eje X (Tiempo)
y = datos(:, 2);  % Segunda columna (B) completa para Veh�culo L�der
w = datos(:, 3);  % Tercera columna (C) completa para Veh�culo Seguidor
z = 80 * ones(size(x));  % L�nea constante de referencia

% Paso 3: Graficar los datos
figure;
plot(x, z, 'r-', 'LineWidth', 2, 'DisplayName', 'Velocidad de referencia');  % L�nea de referencia en rojo
hold on;
plot(x, y, 'b', 'LineWidth', 2, 'DisplayName', 'Veh�culo L�der');  % Veh�culo L�der en azul
plot(x, w, 'g', 'LineWidth', 2, 'DisplayName', 'Veh�culo Seguidor');  % Veh�culo Seguidor en verde

% Paso 4: A�adir t�tulo y nombres a los ejes
title('Gr�fico de Velocidad de Veh�culos Conectados');
xlabel('Tiempo (s)');   % Etiqueta del eje X
ylabel('Velocidad (Pulsos por cada 100ms)');   % Etiqueta del eje Y

% Paso 5: Ajustar la leyenda
legend show;

% Paso 6: Ajustar los valores del eje Y de 1 a 85
ylim([1 85]);  % Limitar el eje Y de 1 a 85

% Paso 7: Mostrar cuadr�cula
grid on;
