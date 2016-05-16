function optArgList = process_arguments(baseArgVarName, optArgName, optArgVarName, varargin_fct)
% process_arguments(baseArgVarName, optArgName, optArgVarName, varargin)
%
%	Processes a list of argument of the type:
%	F(baseArg1, [baseArg2, [baseArg3, [...]]], 'OptArgName', optArg, ...)
%
%	baseArgVarName	cell array of the variable names to be assigned
%					the base arguments
%	optArgName		cell array of the name of the optional arguements
%	optArgVarName	cell array of the variale names to be assigned the
%					optional arguments
%	varargin		pass varargin{:}
%
%	optArgName and optArgVarName must be of the same size


i = 1;
pos = 1;
optArgList = {};
list_i = 1;
varargin = varargin_fct;
while i <= length(varargin)
	
	arg = varargin{i};
	
	% check type
	type = whos('arg');
	if strcmp(type.class, 'char')
		argIdx = find(strcmp(arg, optArgName));
	else
		argIdx = [];
	end
	
	if ~isempty(argIdx)
		if i+1 > length(varargin)
			error('Error in argument list');
        else
            optArgList{list_i} = optArgName{argIdx};
            list_i = list_i + 1;
			assignin('caller', optArgVarName{argIdx}, varargin{i+1});
			i = i + 1;
		end
    elseif pos <= length(baseArgVarName)
		assignin('caller', baseArgVarName{pos}, varargin{i});
		pos = pos + 1;
    else
        disp(['Error: ' arg ' isn''t a valid parameter']);
	end
	
	i = i + 1;
end
