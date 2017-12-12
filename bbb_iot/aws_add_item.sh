#!/bin/bash
echo "Adding Item to DynamoDB"
aws dynamodb put-item --table-name BuyList --item \
'{"Product": {"S": "Milk"}, "NumUnits": {"N": "1"}}' \
--return-consumed-capacity TOTAL

aws dynamodb put-item --table-name BuyList --item \
'{"Product": {"S": "Bread"}, "NumUnits": {"N": "20"}}' \
--return-consumed-capacity TOTAL
