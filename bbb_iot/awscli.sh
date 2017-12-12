#!/bin/bash

aws dynamodb create-table --table-name BuyList --attribute-definitions AttributeName=Product,AttributeType=S AttributeName=NumUnits,AttributeType=N --key-schema AttributeName=Product,KeyType=HASH AttributeName=NumUnits,KeyType=RANGE --provisioned-throughput ReadCapacityUnits=1,WriteCapacityUnits=1
