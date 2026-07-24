function ME446_serialwrite(name,floatval)
%ME446_SERIALWRITE
%   ME446_serialwrite(NAME,FLOATVAL) will modify a variable on the DSP 
%   where NAME is the name of the variable on the DSP, and FLOATVAL is the 
%   value you want to change that variable to.
%
%	For example, to change the float variable "myfloat" on the DSP to a
%	value to 5.5 you would type:
%
%		ME446_serialwrite('myfloat',5.5);
%


% Check for existing serial port objects
existingPorts = serialportfind();

% If any serial ports are found, clean them up
if ~isempty(existingPorts)
    fprintf('length %f\n',length(existingPorts));
    for i = 1:length(existingPorts)
        try
            % Close and delete the serial port object
            delete(existingPorts(i));
        catch
            % Handle errors if the object cannot be deleted
            fprintf('Could not delete serial port: %s\n', existingPorts(i).Port);
        end
    end
end

% Clear the variables
clear existingPorts;


disp("!!Make Sure CCS is configured to Compile for FLASH for this function to work!!")
filename = dir('../CPU_FLASH/*.map');

map = parseMap(strcat('../CPU_FLASH/',filename.name))

memloc = 0;
arrsize = size(map);
found = 1;

for i=1:arrsize(1)
    if strfind(char(map(i,1)),name)
        memloc = char(map(i,2));
        found = 0;
    end
end
if (found == 1)
    exception = MException('MATLAB:VarNotFound','Variable name not found.');
    throw(exception);
end

hex_str = '2A0B33'; % header
hex_str = strcat(hex_str,memloc);
char_str = char(sscanf(hex_str,'%2X').');
s = serialport("COM4",115200);
s.InputBufferSize = 5000;
fopen(s);
fwrite(s,char_str);
fwrite(s,floatval,'float32');
clear s
