function ME446_serial_ListVars()
%ME446_serial_ListVars
%   ME446_serial_ListVars() simple lists the possible variables Matlab has
%	access to inside your DSP application.  Helpful in finding the name of
%	the variable you would like to write to or read from.
%
%		ME446_serial_ListVars();
%
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
