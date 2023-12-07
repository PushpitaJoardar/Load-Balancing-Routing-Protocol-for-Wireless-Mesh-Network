#! /usr/bin/python3
from cProfile import label
from re import L
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import csv
import seaborn as sns
import os

sns.set()
font = {'color':  'black',
        'weight': 'bold',
        'size': 15,
        }


for i in [11]:
    try :
        os.mkdir("nodes_"+str(i))
    except :
        print("directory exists")
    flow_vs_throughput = {}
    flow_vs_delay = {}
    flow_vs_pdr = {}
    flow_vs_plr = {}
    with open("Nodes.dat") as infile :
        for line in infile.read().splitlines():
            vals = line.split("\t")
            print(vals)
            flow_vs_throughput[vals[0]] = vals[1];
            flow_vs_delay[vals[0]] = vals[2];
            flow_vs_pdr[vals[0]] = vals[3];
            flow_vs_plr[vals[0]] = vals[4];
    
        # throughput
        with open("flow_vs_throughput.csv", "w") as outfile:
            csv_writer = csv.writer(outfile)
            csv_writer.writerow(["Flow","Average Throughput(KBps)"])
            for param, values in flow_vs_throughput.items():
                csv_writer.writerow([param,values])
        df = pd.read_csv("flow_vs_throughput.csv")

        font['size']=15
        plt.scatter(df['Flow'], df["Average Throughput(KBps)"], color ='maroon',
                 label="nodes="+str(i))
        plt.plot(df['Flow'], df["Average Throughput(KBps)"], color ='maroon',
                 label="nodes="+str(i))
        plt.xticks(np.arange(10,110,10))
        plt.xlabel('Nodes',fontdict=font)  # Add an x-label to the axes.
        plt.ylabel("Average Throughput(KBps)",fontdict=font)
        font['size']=18
        plt.title("Number of Nodes vs Average Throughput(KBps)",fontdict=font)
        
        plt.savefig("nodes_"+str(i)+"/Number of Nodes vs Average Throughput(KBps).jpg")
        plt.clf()
        if os.path.exists("flow_vs_throughput.csv"):
            os.remove("flow_vs_throughput.csv")

        #end to end delay
        with open("flow_vs_delay.csv", "w") as outfile:
            csv_writer = csv.writer(outfile)
            csv_writer.writerow(["Flow","Delay"])
            for param, values in flow_vs_delay.items():
                csv_writer.writerow([param,values])
        df = pd.read_csv("flow_vs_delay.csv")

        font['size']=15
        plt.scatter(df['Flow'], df["Delay"], color ='maroon',
                 label="nodes="+str(i))
        plt.plot(df['Flow'], df["Delay"], color ='maroon',
                 label="nodes="+str(i))
        plt.xticks(np.arange(10,110,10))
        plt.xlabel('Nodes',fontdict=font)  # Add an x-label to the axes.
        plt.ylabel("Delay",fontdict=font)
        font['size']=18
        plt.title("Number of Nodes vs Delay",fontdict=font)
        
        plt.savefig("nodes_"+str(i)+"/Number of Nodes vs Delay.jpg")
        plt.clf()
        if os.path.exists("flow_vs_delay.csv"):
            os.remove("flow_vs_delay.csv")

        #packet delivery ratio
        with open("flow_vs_pdr.csv", "w") as outfile:
            csv_writer = csv.writer(outfile)
            csv_writer.writerow(["Flow","Packet Delivery Ratio"])
            for param, values in flow_vs_pdr.items():
                csv_writer.writerow([param,values])
        df = pd.read_csv("flow_vs_pdr.csv")

        font['size']=15
        plt.scatter(df['Flow'], df["Packet Delivery Ratio"], color ='maroon',
                 label="nodes="+str(i))
        plt.plot(df['Flow'], df["Packet Delivery Ratio"], color ='maroon',
                 label="nodes="+str(i))
        plt.xticks(np.arange(10,110,10))
        plt.xlabel('Number of Nodes',fontdict=font)  # Add an x-label to the axes.
        plt.ylabel("Packet Delivery Ratio",fontdict=font)
        font['size']=18
        plt.title("Number of Nodes vs Packet Delivery Ratio",fontdict=font)
        
        plt.savefig("nodes_"+str(i)+"/Number of Nodes vs Packet Delivery Ratio.jpg")
        plt.clf()
        if os.path.exists("flow_vs_pdr.csv"):
            os.remove("flow_vs_pdr.csv")
        
        #packet drop ratio
        with open("flow_vs_plr.csv", "w") as outfile:
            csv_writer = csv.writer(outfile)
            csv_writer.writerow(["Flow","Packet Drop Ratio"])
            for param, values in flow_vs_plr.items():
                csv_writer.writerow([param,values])
        df = pd.read_csv("flow_vs_plr.csv")

        font['size']=15
        plt.scatter(df['Flow'], df["Packet Drop Ratio"], color ='maroon',
                 label="nodes="+str(i))
        plt.plot(df['Flow'], df["Packet Drop Ratio"], color ='maroon',
                 label="nodes="+str(i))
        plt.xticks(np.arange(10,110,10))
        plt.xlabel('Number of Nodes',fontdict=font)  # Add an x-label to the axes.
        plt.ylabel("Packet Drop Ratio",fontdict=font)
        
        font['size']=18
        plt.title("Number of Nodes vs Packet Drop Ratio",fontdict=font)
        
        plt.savefig("nodes_"+str(i)+"/Number Of Nodes vs Packet Drop Ratio.jpg")
        plt.clf()
        if os.path.exists("flow_vs_plr.csv"):
            os.remove("flow_vs_plr.csv")

