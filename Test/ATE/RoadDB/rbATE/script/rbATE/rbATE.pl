#*******************************************************************************
#                           Continental Confidential
#                  Copyright (c) Continental, LLC. 2015
#
#      This software is furnished under license and may be used or
#      copied only in accordance with the terms of such license.
#*******************************************************************************
# @file  rbATE.pl
# @brief rbATE    main functions
#
# Change Log:
#      Date                Who             What
#    2015/10/20            Luyao Li        create
#*******************************************************************************
#!C:\Perl\bin\perl.exe

use strict;
use warnings;
use File::Spec;
use File::Basename;
use Cwd;

use Variables;
use Common::Method;
use Var::Var;
use Var::Const;
use Variables;

sub main{
	
	my $count = @$ref_multi_sandbox_git;
	for(my $i=0; $i<$count ; $i++){
		foreach my $keys (keys $$ref_multi_sandbox_git[$i]){
			if($keys !~ /project/){
				$$ref_pjt_info_git[0]{$keys}=$$ref_multi_sandbox_git[$i]{$keys} ;
			}	
		}
		Normal_testSteps();
		$loopID = $loopID + 1;
	}
		
}




sub Normal_testSteps{

	# Fetch source codes from Git
	create_sandbox();
	# process the script in local sandbox
	process_script();
	#  Build server and inVehicle  and run (Microsoft Visual Studio)
	build_project();
	# Send test emails to specified engineers
	send_mail();



}



main();
#print "hello";




