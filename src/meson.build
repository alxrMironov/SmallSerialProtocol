

ssp_dir = include_directories('.')

ssp_lib = library('ssp_lib',
    files('./ssp.c'),
    include_directories: ssp_dir)
	
ssp_dep = declare_dependency(
	link_with: ssp_lib, 
	include_directories: ssp_dir
)

