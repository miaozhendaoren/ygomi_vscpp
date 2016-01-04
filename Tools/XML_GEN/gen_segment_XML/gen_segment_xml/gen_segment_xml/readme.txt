
自动生成的segment的规则：
1.segmentID,laneID,lineID增序切连续
2.默认下所有的segment使用相同的laneID,lineID（也就是说不包含分岔或者更复杂情况）
3.没有考虑海拔
4.默认下presegment,nextsegment与现有的segment连续（也是按照增序排列）
5.输入的参数必须齐备
6.默认下只是支持2个piont每个segment