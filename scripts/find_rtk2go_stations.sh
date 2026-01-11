#!/bin/bash
#
# find_rtk2go_stations.sh - Find nearby RTK2GO base stations
# 
# This script helps you find free RTK base stations from RTK2GO.com
# that are close to your location.
#
# Usage:
#   ./find_rtk2go_stations.sh                    # List all stations
#   ./find_rtk2go_stations.sh 40.78 30.39        # Find stations near lat/lon
#   ./find_rtk2go_stations.sh 40.78 30.39 50     # Find stations within 50km
#   ./find_rtk2go_stations.sh --country TUR      # List stations in Turkey
#
# Author: Davut Can Akbas <akbasdavutcan@gmail.com>
# License: Apache-2.0

set -e

RTK2GO_HOST="rtk2go.com"
RTK2GO_PORT=2101

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║          RTK2GO Free Base Station Finder                   ║${NC}"
    echo -e "${BLUE}║          https://rtk2go.com                                ║${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# Calculate distance between two coordinates using Haversine formula
# Arguments: lat1 lon1 lat2 lon2
# Returns: distance in kilometers
calc_distance() {
    python3 -c "
import math
lat1, lon1, lat2, lon2 = $1, $2, $3, $4
R = 6371  # Earth's radius in km
dlat = math.radians(lat2 - lat1)
dlon = math.radians(lon2 - lon1)
a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
print(f'{R * c:.1f}')
"
}

# Fetch and parse RTK2GO source table
fetch_stations() {
    echo -e "${YELLOW}Fetching station list from RTK2GO...${NC}"
    
    # Try wget first, then curl
    if command -v wget &> /dev/null; then
        STATIONS=$(timeout 30 wget -qO- "http://${RTK2GO_HOST}:${RTK2GO_PORT}" 2>/dev/null || echo "")
    elif command -v curl &> /dev/null; then
        STATIONS=$(timeout 30 curl -s "http://${RTK2GO_HOST}:${RTK2GO_PORT}" 2>/dev/null || echo "")
    else
        echo -e "${RED}Error: Neither wget nor curl found. Please install one.${NC}"
        exit 1
    fi
    
    if [ -z "$STATIONS" ]; then
        echo -e "${RED}Error: Could not fetch station list. Check your internet connection.${NC}"
        exit 1
    fi
    
    # Filter only STR (stream) entries
    echo "$STATIONS" | grep "^STR;"
}

# List stations by country
list_by_country() {
    local country=$1
    echo -e "${GREEN}Stations in ${country}:${NC}"
    echo ""
    printf "%-25s %-30s %-10s %-10s\n" "MOUNTPOINT" "LOCATION" "LAT" "LON"
    echo "--------------------------------------------------------------------------------"
    
    fetch_stations | grep ";${country};" | while IFS=';' read -r type mount location format msgs carrier nav sys country lat lon carrier2 fee snip auth bitrate extra; do
        printf "%-25s %-30s %-10s %-10s\n" "$mount" "$location" "$lat" "$lon"
    done
}

# Find stations near coordinates
find_nearby() {
    local my_lat=$1
    local my_lon=$2
    local max_dist=${3:-100}  # Default 100km
    
    echo -e "${GREEN}Finding stations within ${max_dist}km of (${my_lat}, ${my_lon})...${NC}"
    echo ""
    
    # Create temp file for results
    local tmpfile=$(mktemp)
    
    fetch_stations | while IFS=';' read -r type mount location format msgs carrier nav sys country lat lon carrier2 fee snip auth bitrate extra; do
        # Skip if lat/lon are 0 or empty
        if [ -z "$lat" ] || [ -z "$lon" ] || [ "$lat" = "0.00" ] || [ "$lon" = "0.00" ]; then
            continue
        fi
        
        # Calculate distance
        dist=$(calc_distance "$my_lat" "$my_lon" "$lat" "$lon")
        
        # Check if within range
        if (( $(echo "$dist <= $max_dist" | bc -l) )); then
            echo "${dist}|${mount}|${location}|${country}|${lat}|${lon}|${format}" >> "$tmpfile"
        fi
    done
    
    # Sort by distance and display
    if [ -s "$tmpfile" ]; then
        printf "%-8s %-25s %-25s %-5s %-10s %-10s\n" "DIST" "MOUNTPOINT" "LOCATION" "CTRY" "LAT" "LON"
        echo "===================================================================================="
        
        sort -t'|' -k1 -n "$tmpfile" | while IFS='|' read -r dist mount location country lat lon format; do
            # Color code by distance
            if (( $(echo "$dist <= 35" | bc -l) )); then
                color=$GREEN  # Good for RTK
            elif (( $(echo "$dist <= 50" | bc -l) )); then
                color=$YELLOW  # Marginal
            else
                color=$RED  # Too far for reliable RTK
            fi
            
            printf "${color}%-8s${NC} %-25s %-25s %-5s %-10s %-10s\n" "${dist}km" "$mount" "$location" "$country" "$lat" "$lon"
        done
        
        echo ""
        echo -e "${GREEN}●${NC} <35km: Excellent for RTK"
        echo -e "${YELLOW}●${NC} 35-50km: Usable for RTK"
        echo -e "${RED}●${NC} >50km: Too far for reliable RTK (consider VRS/Network RTK)"
    else
        echo -e "${RED}No stations found within ${max_dist}km${NC}"
        echo ""
        echo "Try increasing the search radius or consider:"
        echo "  1. A paid Network RTK service (TUSAGA-Aktif for Turkey)"
        echo "  2. Setting up your own base station"
        echo "  3. Using RTK2GO with a closer station if available"
    fi
    
    rm -f "$tmpfile"
}

# List all available countries
list_countries() {
    echo -e "${GREEN}Available countries on RTK2GO:${NC}"
    echo ""
    
    fetch_stations | cut -d';' -f9 | sort | uniq -c | sort -rn | head -30
}

# Show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  <lat> <lon>              Find stations near coordinates"
    echo "  <lat> <lon> <radius_km>  Find stations within radius"
    echo "  --country <CODE>         List stations by country code (e.g., TUR, USA, DEU)"
    echo "  --countries              List all available countries"
    echo "  --all                    List all stations"
    echo "  --help                   Show this help"
    echo ""
    echo "Examples:"
    echo "  $0 40.78 30.39           # Find near Sakarya, Turkey"
    echo "  $0 51.50 -0.12 100       # Find within 100km of London"
    echo "  $0 --country TUR         # List Turkish stations"
    echo ""
    echo "For ROS2 integration, use the found mountpoint with ntrip_client:"
    echo ""
    echo "  ros2 run gnss_compass_driver ntrip_client --ros-args \\"
    echo "    -p host:=rtk2go.com \\"
    echo "    -p port:=2101 \\"
    echo "    -p mountpoint:=<MOUNTPOINT> \\"
    echo "    -p username:=your.email@example.com"
}

# Main
print_header

if [ $# -eq 0 ]; then
    show_usage
    exit 0
fi

case "$1" in
    --help|-h)
        show_usage
        ;;
    --countries)
        list_countries
        ;;
    --country)
        if [ -z "$2" ]; then
            echo "Error: Country code required"
            exit 1
        fi
        list_by_country "$2"
        ;;
    --all)
        echo "All RTK2GO stations:"
        fetch_stations | cut -d';' -f2,3,9,10,11 | head -100
        echo "... (showing first 100)"
        ;;
    *)
        # Assume coordinates
        if [ $# -ge 2 ]; then
            find_nearby "$1" "$2" "${3:-100}"
        else
            echo "Error: Need both latitude and longitude"
            show_usage
            exit 1
        fi
        ;;
esac

