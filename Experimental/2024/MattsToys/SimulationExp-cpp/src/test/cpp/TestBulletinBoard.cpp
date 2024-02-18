#include <gtest/gtest.h>

#include "utils/BulletinBoard.h"

class BulletinBoardTest : public testing::Test {
 protected:
  BulletinBoard board;
};

TEST_F(BulletinBoardTest, NoDataWithoutInsertions) {
  EXPECT_FALSE(board.getValue("uninserted").has_value());
}

TEST_F(BulletinBoardTest, SuccessfulRetrievalAfterInsertion) {
  std::string_view key1{"key1"};
  std::string key2{"key2"};
  std::string key3{"key3"};
  std::string key4{"key4"};
  std::string uninsertedKey{"uninserted"};

  board.updateValue(key1, "1");
  board.updateValue(key2, "2");
  board.updateValue(key3, "3");
  board.updateValue(key4, frc::Pose2d(1_m, 1_m, 1_deg));

  EXPECT_TRUE(board.getValue<std::string>(key1).has_value());
  EXPECT_EQ(board.getValue<std::string>(key1).value(), "1");

  EXPECT_TRUE(board.getValue<std::string>(key2).has_value());
  EXPECT_EQ(board.getValue<std::string>(key2).value(), "2");

  EXPECT_TRUE(board.getValue(key3).has_value());
  EXPECT_EQ(std::get<std::string>(board.getValue(key3).value()), "3");
  // Wrong type should yield empty result
  EXPECT_EQ(board.getValue<frc::Pose2d>(key3), std::nullopt);

  EXPECT_TRUE(board.getValue<frc::Pose2d>(key4).has_value());
  EXPECT_EQ(board.getValue<frc::Pose2d>(key4).value(),
            frc::Pose2d(1_m, 1_m, 1_deg));
  // Wrong type should yield empty result
  EXPECT_EQ(board.getValue<std::string>(key4), std::nullopt);

  EXPECT_EQ(board.getValue(uninsertedKey), std::nullopt);
}

TEST_F(BulletinBoardTest, SuccessfulUpdates) {
  std::string_view key1{"key1"};
  std::string key2{"key2"};

  board.updateValue(key1, "1");
  board.updateValue(key2, "2");
  board.updateValue(key1, "one");
  board.updateValue(key2, "two");

  EXPECT_TRUE(board.getValue<std::string>(key1).has_value());
  EXPECT_EQ(board.getValue<std::string>(key1).value(), "one");

  EXPECT_TRUE(board.getValue<std::string>(key2).has_value());
  EXPECT_EQ(board.getValue<std::string>(key2).value(), "two");
}

TEST_F(BulletinBoardTest, SuccessfulDataChange) {
  std::string_view key1{"key1"};

  board.updateValue(key1, frc::Pose2d());
  board.updateValue(key1, "string");

  EXPECT_TRUE(board.getValue<std::string>(key1).has_value());
  EXPECT_EQ(board.getValue<std::string>(key1).value(), "string");
}

TEST_F(BulletinBoardTest, ClearDataValues) {
  std::string_view key1{"key1"};
  std::string key2{"key2"};
  std::string key3{"key3"};
  std::string key4{"key4"};

  board.updateValue(key1, "1");
  board.updateValue(key2, frc::Pose2d(1_m, 1_m, 1_deg));
  board.updateValue(key3, "3");
  board.updateValue(key4, frc::Pose2d(2_m, 2_m, 2_deg));

  board.clearValue(key3);
  board.clearValue(key4);

  // Data not cleared is still there?
  EXPECT_EQ(board.getValue<std::string>(key1).value(), "1");
  EXPECT_EQ(board.getValue<frc::Pose2d>(key2).value(),
            frc::Pose2d(1_m, 1_m, 1_deg));

  // Data that was cleared is gone?
  EXPECT_EQ(board.getValue(key3), std::nullopt);
  EXPECT_EQ(board.getValue(key4), std::nullopt);
}
