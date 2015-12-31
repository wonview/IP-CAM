#!/usr/bin/perl
use strict;

my @tmp_svn_result;
my @tmp_string;
my $svn_status;
my $svn_filename;
my $svn_version;

my @files_modified;
my @files_otherversion;

my $svn_root = ".";
my $svn_root_version;

my @header_defines = (
    "#ifndef _SSV_VERSION_H_",
    "#define _SSV_VERSION_H_",
    "",
    "#define FILE_NAME_SIZE  80",
    "",
);

sub get_version {
    foreach (@_) {
        if($_ =~ m/^Revision: (\d*)/) {
            return $1;
        }
    } 
    # file doesn't exist on svn
    return -1;
}

printf("## script to generate version infomation header ##\n");

# step-0: get root svn number 
$svn_root_version = get_version(qx(svn info $svn_root));

# step-1/2: get modified/otherversion files:
@tmp_svn_result = qx(svn st --verbose $svn_root);
foreach (@tmp_svn_result) {
    if ($_ =~ m/ > /) {     # comment line
        next;
    }
    if ($_ =~ m/^(.{7})/) { # 7 columns of svn state is well defined.
        $svn_status = $1;

        chomp;
        if($svn_status eq "?      ") {
            # printf("## not versioned ##\n");
        }
        elsif ($svn_status eq "       ") {
            s/$svn_status(.*)$/$1/;
            @tmp_string   = split;
            $svn_filename = @tmp_string[3];
            $svn_version  = @tmp_string[0];
            if(@tmp_string[0] ne $svn_root_version) {
                # printf("## other version ##\n");
                push @files_otherversion, sprintf("%s@%s", $svn_filename, $svn_version);
            }
            else {
                # printf("## ok ##\n");
            }
        }
        else {
            s/$svn_status(.*)$/$1/;
            @tmp_string   = split;
            $svn_filename = @tmp_string[$#tmp_string];
            push @files_modified, sprintf("%s:%s", $svn_status, $svn_filename);
        }
    }
}

OUTPUT_HEADER:
# step-3: output header files
if (defined($ARGV[0])) {
    open HEADER, ">", $ARGV[0];
    select HEADER;
}
else {
    print "Error! Please specify header file\n";
}


foreach (@header_defines) {
    printf("%s\n", $_);
}

##
printf("static u32 ssv_root_version = %d;\n\n", $svn_root_version);

##
printf("static char files_modified[][FILE_NAME_SIZE] = {\n");
foreach (@files_modified) {
    printf("    \"%s\",\n", $_);
}
printf("};\n\n");


printf("static char files_otherversion[][FILE_NAME_SIZE] = {\n");
foreach (@files_otherversion) {
    printf("    \"%s\",\n", $_);
}
printf("};\n\n");

printf("#endif\n");
