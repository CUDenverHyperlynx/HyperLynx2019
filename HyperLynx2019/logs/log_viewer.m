clear all; close all; clc;

%% Load logfile into array
file_name = input('Filename: ', 's');
datatype = input('Data label: ', 's');

file = fopen(file_name, 'r');
x = readtable(file_name);
[rows, columns] = size(x);
for i = 1:(rows-1)
    data(i,:)= textscan(file, ['%s\t%f\t%f\t%f\n'], 'HeaderLines',1);
end

fclose(file);
fprintf("Length of file: %d\n", length(data));
j = 1;
%datatype = 'Brake_Pressure';

%% Load specific datatype into array for plotting
for i = 1:length(data)
    if datatype == string(data{i,1})
        spec_data(j,1) = data{i,2};
        spec_data(j,2) = data{i,4};
        j = j + 1;
    end
end

%% Plot
plot(spec_data(:,2), spec_data(:,1));
title(datatype);