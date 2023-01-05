function [dout]=loadadc5ch(filnam)
% load  5-channel analog logger data file
% file is structure is:
%  32-bit unsigned integer unix seconds
%  16-bit unsigned integer spare (used for microseconds between samples)
%  5 x 16-bit unsigned integer ADC counts
%  2/5/21  MJB

if nargin == 0
    [raw_name,temp,filterindex]=uigetfile('*.dat','Load ADC 5-channel Logger File');
    filnam=[temp raw_name];
else 
    filnam = [filnam];  % handy for functions that scan a directory
end

disp(sprintf('Reading data from %s',filnam));
fid = fopen(filnam,'r');
fseek(fid,0,'eof'); % move to end of file
pos2 = ftell(fid); % pos2 is overall length of file
frewind(fid); % move back to beginning of file

%  each record is 16 bytes as described above

nrecords=floor((pos2)/16); % number of records
disp(sprintf('%d records of data in the file',nrecords));


%read unix seconds
fseek(fid,0,'bof');
dout.secs = fread(fid,nrecords,'uint32',12); % skip  12 bytes

fseek(fid,4,'bof'); % rewind to beginning of spare field
% next read spare
dout.spare = fread(fid,nrecords,'uint16',14); % skip  14

fseek(fid,6,'bof'); % start at beginning of CH0
dout.dshot_cmd = fread(fid,nrecords,'uint16',14);

fseek(fid,8,'bof'); % start at beginning of CH1
dout.deg = fread(fid,nrecords,'uint16',14);

fseek(fid,10,'bof'); % start at beginning of CH2
dout.volt = fread(fid,nrecords,'uint16',14);

fseek(fid,12,'bof'); % start at beginning of CH3
dout.amp = fread(fid,nrecords,'uint16',14);

fseek(fid,14,'bof'); % start at beginning of CH4
dout.rpm = fread(fid,nrecords,'uint16',14);

%   sptr->avals[0] = (uint16_t)ESCPID_comm.cmd[i];
%   sptr->avals[1] = (uint16_t)ESCPID_comm.deg[i];
%   sptr->avals[2] = (uint16_t)ESCPID_comm.volt[i]*0.01;
%   sptr->avals[3] = (uint16_t)ESCPID_comm.amp[i]*0.01;
%   sptr->avals[4] = (uint16_t)ESCPID_comm.rpm[i]*11.11;

fclose(fid);
plot(dout.dshot_cmd); % check noise in 2.5V reference sample

% now we determine the sample rate by seeing how many samples 
% occur between changes in the seconds value
tp = diff(dout.secs);  %  should be zero or one
tidx = find(tp); % get the indices of non-zero values in tp
samprate  = mean(diff(tidx)); %
disp(sprintf('Recovered sample rate is %6.2f samples per second',samprate));
dout.samprate = samprate;
end