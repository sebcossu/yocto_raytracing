# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.hwlib.Debug:
/Users/seb/Desktop/out/bin/Debug/libhwlib.a:
	/bin/rm -f /Users/seb/Desktop/out/bin/Debug/libhwlib.a


PostBuild.raytrace.Debug:
PostBuild.hwlib.Debug: /Users/seb/Desktop/out/bin/Debug/raytrace
/Users/seb/Desktop/out/bin/Debug/raytrace:\
	/Users/seb/Desktop/out/bin/Debug/libhwlib.a
	/bin/rm -f /Users/seb/Desktop/out/bin/Debug/raytrace


PostBuild.hwlib.Release:
/Users/seb/Desktop/out/bin/Release/libhwlib.a:
	/bin/rm -f /Users/seb/Desktop/out/bin/Release/libhwlib.a


PostBuild.raytrace.Release:
PostBuild.hwlib.Release: /Users/seb/Desktop/out/bin/Release/raytrace
/Users/seb/Desktop/out/bin/Release/raytrace:\
	/Users/seb/Desktop/out/bin/Release/libhwlib.a
	/bin/rm -f /Users/seb/Desktop/out/bin/Release/raytrace


PostBuild.hwlib.MinSizeRel:
/Users/seb/Desktop/out/bin/MinSizeRel/libhwlib.a:
	/bin/rm -f /Users/seb/Desktop/out/bin/MinSizeRel/libhwlib.a


PostBuild.raytrace.MinSizeRel:
PostBuild.hwlib.MinSizeRel: /Users/seb/Desktop/out/bin/MinSizeRel/raytrace
/Users/seb/Desktop/out/bin/MinSizeRel/raytrace:\
	/Users/seb/Desktop/out/bin/MinSizeRel/libhwlib.a
	/bin/rm -f /Users/seb/Desktop/out/bin/MinSizeRel/raytrace


PostBuild.hwlib.RelWithDebInfo:
/Users/seb/Desktop/out/bin/RelWithDebInfo/libhwlib.a:
	/bin/rm -f /Users/seb/Desktop/out/bin/RelWithDebInfo/libhwlib.a


PostBuild.raytrace.RelWithDebInfo:
PostBuild.hwlib.RelWithDebInfo: /Users/seb/Desktop/out/bin/RelWithDebInfo/raytrace
/Users/seb/Desktop/out/bin/RelWithDebInfo/raytrace:\
	/Users/seb/Desktop/out/bin/RelWithDebInfo/libhwlib.a
	/bin/rm -f /Users/seb/Desktop/out/bin/RelWithDebInfo/raytrace




# For each target create a dummy ruleso the target does not have to exist
/Users/seb/Desktop/out/bin/Debug/libhwlib.a:
/Users/seb/Desktop/out/bin/MinSizeRel/libhwlib.a:
/Users/seb/Desktop/out/bin/RelWithDebInfo/libhwlib.a:
/Users/seb/Desktop/out/bin/Release/libhwlib.a:
