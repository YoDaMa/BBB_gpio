#!/bin/bash


itemjson="{\"Product\": {\"S\": \"Egg\"},\"Timestamp\": {\"S\": \"$(date)\"},\"Quantity\": {\"N\": \"12\"}}"

echo $itemjson > eggitem.json


aws dynamodb put-item --table-name BuyList \
    --item file://eggitem.json 


