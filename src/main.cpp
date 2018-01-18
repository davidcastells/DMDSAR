/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   main.cpp
 * Author: dcr
 *
 * Created on January 11, 2018, 3:39 PM
 */

#include <QApplication>

#include <stdio.h>

#include "Protractor.h"




int main(int argc, char *argv[])
{
    int ret;
    
    
    // initialize resources, if needed
    // Q_INIT_RESOURCE(resfile);
    
    QApplication app(argc, argv);

    
    // create and show your widgets here
    Protractor indicator;
    
    indicator.parseOptions(argc, argv);
    indicator.show();
  
    ret = app.exec();
    
    printf("Exiting App\n");
    
    return ret;
}
