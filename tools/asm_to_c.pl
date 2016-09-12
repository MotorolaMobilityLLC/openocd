#!/usr/bin/perl
#
# Copyright (c) 2016 Motorola Mobility LLC
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the
# Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
# Description:
#    Converts an object file to a C uint8_t array for use in the source code.
#
# Usage:
#    asm_to_c <object_file> [indent spaces]
#
use strict;
use warnings;
use Text::Tabs;

my $OBJDUMP="arm-none-eabi-objdump -d";

my $infile = $ARGV[0];
if (! -e $infile)
{
    die "Unable to open input file $infile.\n";
}

my $indent = 4;
if (defined $ARGV[1])
{
    $indent = $ARGV[1];
}

# Get an array containing the assembly code and object code.
my @obj_code = `$OBJDUMP $infile`;
chomp @obj_code;

# Replace tabs with spaces.
$tabstop = 4;
my @obj_code_no_tab = expand(@obj_code);

my $longest_line = 0;
# Find the longest line for the end comment. */
foreach my $line (@obj_code_no_tab)
{
    if (length $line > $longest_line)
    {
        $longest_line = length $line;
    }
}

sub indent
{
    return " " x $indent;
}

my $comment_length = $longest_line;
# Loop through the array parsing the lines.
print indent() . "{\n";
foreach my $line (@obj_code_no_tab)
{
    # Check for a label line in the form:
    # 00000000 <flash_wait_busy_1-0xc>:
    if ($line =~ /[0-9a-fA-F]+\s+\<(?<address>\S+)\>:/)
    {
        print indent() . "                               /* $+{address}:" . " " x ($comment_length - length $+{address}) . "*/\n";
    }
    # Check for a line with just data in the form:
    #   64:	000411aa 	.word	0x000411aa
    elsif ($line =~ /\s*[0-9a-fA-F]:\s+(?<byte_4>[0-9a-fA-F]{2})(?<byte_3>[0-9a-fA-F]{2})(?<byte_2>[0-9a-fA-F]{2})(?<byte_1>[0-9a-fA-F]{2})\s+\.word/)
    {
        my $code = $line . " " x ($comment_length - length $line);
        print indent() . "    0x$+{byte_1}, 0x$+{byte_2}, 0x$+{byte_3}, 0x$+{byte_4},    /* $code */\n";
    }
    # Check for a line with double word data in the form:
    # 2:	f015 4f00 	tst.w	r5, #2147483648	; 0x80000000
    elsif ($line =~ /\s*[0-9a-fA-F]:\s+(?<byte_2>[0-9a-fA-F]{2})(?<byte_1>[0-9a-fA-F]{2})\s+(?<byte_4>[0-9a-fA-F]{2})(?<byte_3>[0-9a-fA-F]{2})\s+(?<code>.*)/)
    {
        my $code = $line . " " x ($comment_length - length $line);
        print indent() . "    0x$+{byte_1}, 0x$+{byte_2}, 0x$+{byte_3}, 0x$+{byte_4},    /* $code */\n";
    }
    # Check for a line with a single word data in the form:
    # 0:	695d      	ldr	r5, [r3, #20]
    elsif ($line =~ /\s*[0-9a-fA-F]+:\s+(?<byte_2>[0-9a-fA-F]{2})(?<byte_1>[0-9a-fA-F]{2})\s+(?<code>.*)/)
    {
        my $code = $line . " " x ($comment_length - length $line);
        print indent() . "    0x$+{byte_1}, 0x$+{byte_2},                /* $code */\n";
    }
}
print indent() . "};\n";
