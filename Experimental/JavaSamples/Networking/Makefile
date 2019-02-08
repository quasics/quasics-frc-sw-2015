##############################################################################
# This makefile originally created by gmakemake, Fri Feb  8 08:46:13 EST 2019
# 
# Note: documentation for (GNU) make can be found online at 
#       http://www.gnu.org/software/make/manual/make.html
##############################################################################

####
#### Special (built-in) targets for make/gmake
####


# The following target stops commands from being displayed during execution
.SILENT:

# The following target is only supported for GNU make, and indicates that 
# certain other targets don't actually produce a result in the file system 
# an object file, or an executable program), and instead they are just 
# 'convenience targets' to support what the programmer is doing.
.PHONY: clean purge reallyclean all objs archive unarchive help beautify 


####
#### Macros (variables) that will be used by make/gmake
####

# The various source files for our program(s)
MAIN_JAVA_FILES  =  BroadcastGUI.java BroadcastingClient.java
OTHER_JAVA_FILES = 

# Other files for your project, such as a 'ReadMe', etc.
OTHER_FILES = 

# Compilers and compiler settings


# Settings for code beautification (via 'astyle')
ASTYLE = astyle
ASTYLE_FLAGS += -A2 --add-brackets --indent=spaces=4 --break-closing-brackets
ASTYLE_FLAGS += --convert-tabs -C -S -N -w --pad-header --add-brackets 
ASTYLE_FLAGS += --align-pointer=name --add-one-line-brackets 
ASTYLE_FLAGS += --max-code-length=80

JAVAC      ?= javac 
JAVA       ?= java
JAVADOC    ?= javadoc

# Optional: Set the following flag to build for a particular JVM level
# JAVASOURCE = 1.6

CLASSPATH_SEP ?= :

ifneq ($(JAVASOURCE),)
JAVACFLAGS += -source $(JAVASOURCE) 
endif
JAVACFLAGS += -sourcepath . -d "$(CLASSDIR)" 
JAVACFLAGS += -classpath "$(CLASSPATH)$(CLASSPATH_SEP)$(CLASSDIR)"
JAVACFLAGS += -deprecation 
JAVACFLAGS += -Xlint:all

JAVAFLAGS  +=  -classpath "$(CLASSPATH)$(CLASSPATH_SEP)$(CLASSDIR)"
JAVAFLAGS  +=  -Djava.net.preferIPv4Stack=true 

JDOCFLAGS  += -classpath "$(CLASSPATH)$(CLASSPATH_SEP)$(CLASSDIR)"
JDOCFLAGS  += -d "$(JDOCDIR)" 
JDOCFLAGS  += -private -use -version -author -serialwarn 

# The subdirectory where class files will be stored
CLASSDIR = classes

# The subdirectory where Javadoc files should be placed
JDOCDIR  = html

####
# Other environmental detection/set-up
#

# The target platform.  This will default to whatever the 'uname' command 
# reports as the hardware/OS name for the current system when 'make' is 
# run, but can be manually overridden if you're using a cross-compiler, 
# etc.
ARCH := $(shell uname -m)
OS := $(shell uname)


#-------------------------------------------------------------------
# The values for the rest of the macro variables are generated from 
# the settings above (or are typically "well-known" values).
#-------------------------------------------------------------------

# Generate the full list(s) of typed source files under our concern
JAVA_FILES   = $(MAIN_JAVA_FILES) $(OTHER_JAVA_FILES)

# Generate the master list of *all* source files under our concern
SOURCEFILES += $(JAVA_FILES) 

# The names of the applications are derived from the 'base names' 
# for their respective source files (without extensions)
MAIN_JAVA_BASES =   $(basename $(MAIN_JAVA_FILES))

# Figure out the 'base names' of the other source files
OTHER_JAVA_BASES =   $(basename $(OTHER_JAVA_FILES))

# Here's the (type-specific) list of base names for all source files
JAVA_BASES = $(MAIN_JAVA_BASES) $(OTHER_JAVA_BASES)

CLASSFILES =   $(MAIN_JAVA_BASES:%=$(CLASSDIR)/%.class) \
               $(OTHER_JAVA_BASES:%=$(CLASSDIR)/%.class)


####
#### Implicit rules, telling make/gmake how to create one kind of file 
#### from another kind of file (e.g. how to create "foo.o" from 
#### "foo.cpp")
####

# Compiling Java code in ".java" files: note that the compiler will handle 
# any sub-directories for packages
$(CLASSDIR)/%.class: %.java
	@echo "$@"
	@mkdir -p "$(CLASSDIR)"
	$(JAVAC) $(JAVACFLAGS) $*.java


####
#### Explicit targets
####
all:	$(CLASSFILES) $(APPS) 

help:
	@echo "Targets include:"
	@echo "all clean purge archive unarchive classes beautify " | fmt -65 | sed -e 's/^/	/'; \
	echo "" ; \
	echo "Application targets include:" ; \
	echo "$(APPS) " | fmt -65 | sed -e 's/^/	/'

classes: $(CLASSFILES)

jdocs: 
	$(JAVADOC) $(JDOCFLAGS) *.java 

archive:
	tar cvf Archive.tar $(SOURCEFILES) $(OTHER_FILES) [mM]akefile 
	compress Archive.tar

unarchive:
	zcat Archive.tar | tar xvf - 

rebuild: clean all

clean:
	$(RM) $(OBJDIR)/*.$(OBJEXT) 
	$(RM) $(CLASSFILES) *.class 
	$(RM) $(APPS) *.exe $(EXEDIR)/*.exe core *.exe.stackdump 

purge reallyclean: clean
	if [ "$(EXEDIR)" != "." ] ; then $(RM) -r $(EXEDIR) ; fi
	$(RM) -r $(OBJDIR) $(CLASSDIR) $(JDOCDIR) 
	$(RM) $(DEPENDENCY_FILE) 

#
# Rules to *run* our applications (if new apps are added, create a 
# new item like those below, for the new application)
#

BroadcastGUI:	$(CLASSFILES) 
	$(JAVA) $(JAVAFLAGS) BroadcastGUI 

BroadcastingClient:	$(CLASSFILES) 
	$(JAVA) $(JAVAFLAGS) BroadcastingClient 

####
# Code beautification targets
#
beautify:
	for f in $(SOURCEFILES) ; do $(ASTYLE) $(ASTYLE_FLAGS) $$f ; done


####
#### Dependency management (courtesy of some intelligence in gcc.... ;-)
####


-include $(DEPS_FILES)


