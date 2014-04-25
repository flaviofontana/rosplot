rosplot
=============

Tool to plot data from ros. Derived from the rqt_plot. 
* autocompletion works with namespaces
* uses less cpu than matlibplot
* xaxis shows time
* zoom fixed
* rad2deg conversion 


##instalation
```
./scripts/install.sh
```

##usage
```
rosplot /nanoquad/state/position/x /nanoquad/state/position/y
```
or
``` 
rosplot /nanoquad/state/position/x:y:z
```
note that the topics can be autocompleted.

