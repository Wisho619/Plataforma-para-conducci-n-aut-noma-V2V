% Paso 1: Leer los datos desde el archivo Excel
datos = xlsread('TL.csv', 1, 'A1:C348');  % Lee datos de las columnas A, B, y C hasta la fila 348

% Paso 2: Separar las columnas
x = datos(:, 1);  % Primera columna (A) completa para el eje X (Tiempo)
y = datos(:, 2);  % Segunda columna (B) completa para Vehículo Líder
w = datos(:, 3);  % Tercera columna (C) completa para Vehículo Seguidor
z = 80 * ones(size(x));  % Línea constante de referencia

% Paso 3: Graficar los datos
figure;
plot(x, z, 'r-', 'LineWidth', 2, 'DisplayName', 'Velocidad de referencia');  % Línea de referencia en rojo
hold on;
plot(x, y, 'b', 'LineWidth', 2, 'DisplayName', 'Vehículo Líder');  % Vehículo Líder en azul
plot(x, w, 'g', 'LineWidth', 2, 'DisplayName', 'Vehículo Seguidor');  % Vehículo Seguidor en verde

% Paso 4: Añadir título y nombres a los ejes
title('Gráfico de Velocidad de Vehículos Conectados');
xlabel('Tiempo (s)');   % Etiqueta del eje X
ylabel('Velocidad (Pulsos por cada 100ms)');   % Etiqueta del eje Y

% Paso 5: Ajustar la leyenda
legend show;

% Paso 6: Ajustar los valores del eje Y de 1 a 85
ylim([1 85]);  % Limitar el eje Y de 1 a 85

% Paso 7: Mostrar cuadrícula
grid on;
