%SMEXPORTONSHAPE_16B Export an Onshape CAD model as an XML multibody description
% file and a set of STEP body geometry files.
% 
% multibodyDescriptionFile = smexportonshape_16b(assemblyURL) generates an XML
% multibody description file and a set of STEP body geometry files for an
% Onshape CAD assembly model. The files provide the smimport function the data
% and geometries that it needs to generate an equivalent Simscape Multibody
% model. The XML file conforms to the published Simscape Multibody XML schema.
% 
% assemblyURL is the web address of the Onshape assembly model to import. To
% obtain the URL, open the Onshape assembly model, select the assembly tab, and
% copy the URL from the web browser. The assembly model need not belong to your
% Onshape account if it is shared with you or made public. Only assembly models
% are supported. Do not use this function to export individual Onshape parts.
% 
% multibodyDescriptionFile is the name of the XML file that contains the
% structure and data of the Onshape assembly model. You can later specify this
% name as an argument of the smimport function to import the model. The file
% name is derived from the Onshape assembly name. All files are saved in the
% current MATLAB folder.
% 
% multibodyDescriptionFile = smexportonshape_16b(assemblyURL, Name, Value) adds a
% name-value pair argument to specify the folder in which to save the XML and
% STEP files for the model.
% 
% Name-Value Pair Arguments: 
% ---------------------------- 
% 'FolderPath'      Path of the folder, specified as a character vector or
% string, in which to save the XML and STEP files generated during export. The
% path can be absolute or relative. You must have write privileges to the folder
% in order to save the files there. Example: 'C:\Documents\Models'
% 
% Copyright 2016 The MathWorks, Inc.
% P-Coded MATLAB Function

