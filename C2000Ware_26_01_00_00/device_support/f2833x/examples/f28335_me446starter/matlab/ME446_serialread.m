function [out3] = ME446_serialread(name,varsize)
%ME446_SERIALREAD
%   N = ME446_serialread(NAME,VARSIZE) will return data from the DSP declared
%	in either the ".my_arrs" or ".my_vars" data sections where NAME refers to
%	the name of the variable or array on the DSP, and varsize refers to the
%	size (number of 32-bit words) in the variable or array.
%
%	For example, to read the float variable "myfloat" from the DSP and store
%	it into a Matlab single variable called "mydata" you would type:
%
%		mydata = ME446_serialread('myfloat',1);
%
%	To read the first 1000 entries in the float array "myfloatarray" 
%	and store it into a Matlab single variable array called "myarray" 
%	you would type:	
%
%		myarray = ME446_serialread('myfloatarray',1000);
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

hexsize = dec2hex(varsize,4);
hex_str = '2A0932'; % header
hex_str = strcat(hex_str,memloc,hexsize);
char_str = char(sscanf(hex_str,'%2X').');


s = serialport("COM4",115200);
s.InputBufferSize = 15000;
fopen(s);
fwrite(s,char_str);

count = 0;
while 1
    inchar = fread(s,1);
    if inchar == 42

        inchar = fread(s,1);
        inchar = fread(s,1);

        if inchar == 51
            out3 = fread(s,varsize,'float32');
            break;
        end
    end
    count = count + 1;
    if count > 100
        break;
    end
end

clear s
