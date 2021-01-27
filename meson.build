
###
#	Small serial protocol 
#

project('ssp', 'c')

unity_dep = dependency('unity', fallback : ['unity', 'unity_dep'])

subdir('.\src')

test('Running SSP Test', 
	executable(
		'SSP Test', 
		'./test/test.c', 
		dependencies: [ ssp_dep, unity_dep ]))