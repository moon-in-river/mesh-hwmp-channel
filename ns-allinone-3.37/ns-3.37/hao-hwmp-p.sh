#! /bin/bash

gnome-terminal  --window --title="1" -e 'bash -c "./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=1\";exec bash"' \
--tab --title="2" -e 'bash -c "sleep 1s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=2\";exec bash"' \
--tab --title="3" -e 'bash -c "sleep 2s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=3\";exec bash"' \
--tab --title="4" -e 'bash -c "sleep 3s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=4\";exec bash"' \
--tab --title="5" -e 'bash -c "sleep 4s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=5\";exec bash"' \
--tab --title="6" -e 'bash -c "sleep 5s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=6\";exec bash"' \
--tab --title="7" -e 'bash -c "sleep 6s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=7\";exec bash"' \
--tab --title="8" -e 'bash -c "sleep 7s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=8\";exec bash"' \
--tab --title="9" -e 'bash -c "sleep 8s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=9\";exec bash"' \
--tab --title="10" -e 'bash -c "sleep 9s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=10\";exec bash"' \
--tab --title="11" -e 'bash -c "sleep 1s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=11\";exec bash"' \
--tab --title="12" -e 'bash -c "sleep 2s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=12\";exec bash"' \
--tab --title="13" -e 'bash -c "sleep 3s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=13\";exec bash"' \
--tab --title="14" -e 'bash -c "sleep 4s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=14\";exec bash"' \
--tab --title="15" -e 'bash -c "sleep 5s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=15\";exec bash"' \
--tab --title="16" -e 'bash -c "sleep 6s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=16\";exec bash"' \
--tab --title="17" -e 'bash -c "sleep 7s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=17\";exec bash"' \
--tab --title="18" -e 'bash -c "sleep 8s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=18\";exec bash"' \
--tab --title="19" -e 'bash -c "sleep 9s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=19\";exec bash"' \
--tab --title="20" -e 'bash -c "sleep 1s;./ns3 run \"scratch/hao-hwmp-p.cc --RngRun=20\";exec bash"' \
