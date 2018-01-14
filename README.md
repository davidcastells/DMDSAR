# DMDSAR

We try to augment the real world with DMD-based mobile projectors. 

I encourage that you cite our paper:

<pre>David Castells-Rufas, Francesc Bravo-Montero, Byron Quezada-Benalcazar, Jordi Carrabina.
"Automatic real-time tilt correction of DMD-based displays for augmented reality applications".
In Society of Photo-Optical Instrumentation Engineers (SPIE) Conference Series, vol. 10546. 2018.
</pre>


## Building the code

I use Netbeans 8.2 in Windows 10 (64bit), and tried to use MSYS2 (http://www.msys2.org/) to have a Qt development platform

Although the Qt interface building tools work, the Makefile fails to compile. I presume it is something related with PATH, 
because the error complains about some cygwin stuff which is never referenced in the project configuration (but which I 
have installed in my system).

So (by now) I am opening a MSYS2 console and compile manually by

```
make -f Makefile CONF=Debug QMAKE=/C/msys64/mingw64/bin/qmake.exe
```

## Executing the code

From the MSYS2 console
```
dist/Debug/MSYS2-Windows/DMDSAR.exe --emulate-IMU
```

You can use the following parameters

--help          shows some hwlp
--emulate-IMU   uses an emulated IMU, useful for debugging
