#!/bin/bash

# Five-Floor Building Wi-Fi HaLow Mesh Network Simulation
# Execution Script

# This script runs the five-floor building simulation with all three routing protocols:
# AODV, AODVv2, and OLSR

# Usage:
# ./run-five-floor-building.sh [protocol]
# 
# If no protocol specified, runs all three
# Valid protocols: aodv, aodvv2, olsr

# Configuration
SCRATCH_DIR="scratch"
WAF="./waf"

# Results directory
RESULTS_DIR="results"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to check if build exists
check_build() {
    if [ ! -x "$WAF" ]; then
        echo -e "${RED}Error: waf not found in the repository root${NC}"
        echo -e "${YELLOW}Please run this script from the repository root${NC}"
        exit 1
    fi
}

# Function to run simulation with specific protocol
run_simulation() {
    local protocol=$1
    local protocol_name=""
    
    case $protocol in
        aodv)
            protocol_name="aodv"
            ;;
        aodvv2)
            protocol_name="aodvv2"
            ;;
        olsr)
            protocol_name="olsr"
            ;;
        *)
            echo -e "${RED}Error: Unknown protocol '$protocol'${NC}"
            echo -e "Valid protocols: aodv, aodvv2, olsr"
            exit 1
            ;;
    esac
    
    echo -e "${BLUE}====================================${NC}"
    echo -e "${BLUE}Running ${protocol_name} Simulation${NC}"
    echo -e "${BLUE}====================================${NC}"
    
    # Create results directory
    mkdir -p "$RESULTS_DIR/$protocol_name/pcap"
    
    # Run simulation
    echo -e "${YELLOW}Starting simulation...${NC}"
    $WAF --disable-python --run "$SCRATCH_DIR/five-floor-building --protocol=$protocol --pcap=true --flowmon=true"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}${protocol_name} simulation completed successfully!${NC}"
    else
        echo -e "${RED}${protocol_name} simulation failed!${NC}"
        exit 1
    fi
}

# Function to verify results
verify_results() {
    local protocol=$1
    local protocol_name=""
    
    case $protocol in
        aodv) protocol_name="aodv" ;;
        aodvv2) protocol_name="aodvv2" ;;
        olsr) protocol_name="olsr" ;;
    esac
    
    echo -e "${BLUE}====================================${NC}"
    echo -e "${BLUE}Verifying ${protocol_name} Results${NC}"
    echo -e "${BLUE}====================================${NC}"
    
    # Check PCAP files
    local pcap_count=$(ls $RESULTS_DIR/$protocol_name/pcap/*.pcap 2>/dev/null | wc -l)
    if [ $pcap_count -eq 20 ]; then
        echo -e "${GREEN}✓ All 20 PCAP files generated${NC}"
    else
        echo -e "${RED}✗ Expected 20 PCAP files, found $pcap_count${NC}"
    fi
    
    # Check PCAP file sizes
    local empty_pcap=$(find $RESULTS_DIR/$protocol_name/pcap/ -name "*.pcap" -size 0 2>/dev/null | wc -l)
    if [ $empty_pcap -eq 0 ]; then
        echo -e "${GREEN}✓ No empty PCAP files${NC}"
    else
        echo -e "${RED}✗ Found $empty_pcap empty PCAP files${NC}"
    fi
    
    # Check FlowMonitor file
    if [ -f "$RESULTS_DIR/$protocol_name/flowmon.xml" ]; then
        local flow_size=$(stat -c%s $RESULTS_DIR/$protocol_name/flowmon.xml)
        if [ $flow_size -gt 100 ]; then
            echo -e "${GREEN}✓ FlowMonitor XML file generated ($flow_size bytes)${NC}"
        else
            echo -e "${RED}✗ FlowMonitor XML file is too small ($flow_size bytes)${NC}"
        fi
    else
        echo -e "${RED}✗ FlowMonitor XML file not found${NC}"
    fi
    
    echo ""
}

# Function to generate summary report
generate_summary() {
    echo -e "${BLUE}====================================${NC}"
    echo -e "${BLUE}Simulation Summary${NC}"
    echo -e "${BLUE}====================================${NC}"
    
    for protocol in aodv aodvv2 olsr; do
        local protocol_name=""
        case $protocol in
            aodv) protocol_name="aodv" ;;
            aodvv2) protocol_name="aodvv2" ;;
            olsr) protocol_name="olsr" ;;
        esac
        
        if [ -d "$RESULTS_DIR/$protocol_name" ]; then
            local pcap_count=$(ls $RESULTS_DIR/$protocol_name/pcap/*.pcap 2>/dev/null | wc -l)
            local flow_file="$RESULTS_DIR/$protocol_name/flowmon.xml"
            
            if [ -f "$flow_file" ]; then
                local flow_size=$(stat -c%s $flow_file)
                echo -e "${protocol_name}: $pcap_count PCAP files, FlowMonitor: $flow_size bytes"
            else
                echo -e "${protocol_name}: $pcap_count PCAP files, FlowMonitor: NOT FOUND"
            fi
        else
            echo -e "${protocol_name}: RESULTS DIRECTORY NOT FOUND"
        fi
    done
    
    echo ""
}

# Main execution
main() {
    check_build
    
    if [ $# -eq 0 ]; then
        # Run all protocols
        echo -e "${GREEN}Running all three routing protocols...${NC}"
        echo ""
        
        for protocol in aodv aodvv2 olsr; do
            run_simulation $protocol
            verify_results $protocol
        done
        
        generate_summary
    else
        # Run specific protocol
        run_simulation $1
        verify_results $1
    fi
    
    echo -e "${GREEN}====================================${NC}"
    echo -e "${GREEN}All simulations completed!${NC}"
    echo -e "${GREEN}====================================${NC}"
}

# Run main function
main "$@"
